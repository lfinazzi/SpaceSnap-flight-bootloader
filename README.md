# UNSAM SpaceSnap (USS) — Bootloader

Bootloader for the **UNSAM SpaceSnap** imaging payload. Runs on every reset of the payload MCU aboard **UNSAMSat-01**, validates the application image in internal flash against CRC32 checksums stored in FRAM, restores it from one of two independent FRAM backup copies if validation fails, and hands off execution to the application. If neither the flash image nor either FRAM backup copy produces a valid, jumpable image, the bootloader halts permanently.

---

## Overview

| Item | Details |
|------|---------|
| **MCU** | STM32F217ZGTX (Arm Cortex-M3, 120 MHz) |
| **Image validation** | CRC32 (zlib-compatible, polynomial `0xEDB88320`) of the application region in flash against a checksum stored in FRAM |
| **Backup copies** | Two independent 128 KB copies in FRAM (copy A at `0x1C0000`, copy B at `0x1E0000`) |
| **Application entry point** | `0x08004000` (Sector 1) |
| **Bootloader footprint** | Sector 0 only (16 KB); linker script enforces this at link time |
| **Debug UART** | UART4, 460800 baud; compiled out entirely via `BOOTLOADER_DEBUG_LOG 0` for flight builds |
| **Watchdog** | IWDG, ~5.6–8.2 s window; refreshed throughout all long flash operations |
| **Build toolchain** | STM32CubeIDE (Eclipse CDT / GCC ARM) |

---

## Features

- **CRC32 image validation** — computes a zlib-compatible CRC32 over the application region in flash and compares it against the checksum stored in the FRAM backup A header before ever jumping. The same algorithm is used by the application firmware's `CMD_BackupFirmware` command to write the stored checksum, ensuring byte-perfect agreement.
- **Dual FRAM backup with fallback** — on a flash CRC mismatch, attempts to restore from copy A first. If copy A's restore also produces a CRC mismatch, falls back to copy B. Both copies must independently fail before the bootloader halts — requiring two independent FRAM corruption events to render the payload unbootable.
- **Flash-first fast path** — on a clean boot where the flash image is already valid, the bootloader reads both backup headers, validates the flash CRC against copy A's stored value, and jumps immediately without any FRAM restore operation. This is the common case in normal mission operation.
- **Vector-table sanity checks** — before every jump attempt, confirms the application's initial stack pointer (at `APP_ADDRESS + 0`) falls within `[0x20000000, 0x20020000]` (valid SRAM range) and that the reset handler address (at `APP_ADDRESS + 4`) has the Thumb bit set. Catches blank/erased flash without requiring a prior CRC check to have passed.
- **Watchdog-aware flash operations** — `HAL_IWDG_Refresh()` is called between every sector erase in `Flash_ErasePages()` and between every 256-byte program chunk in `RestoreAppFromFRAM()`. A full restore of a 91 KB image across multiple sectors cannot trip the IWDG mid-operation.
- **Fail-safe halt** — if no valid, jumpable image can be obtained from flash or either FRAM backup, `Error_Handler()` disables interrupts and halts in an infinite loop. There is no recovery beyond this point without a new flash via ST-LINK.
- **Flight/debug build separation** — the `FLIGHT_BUILD` macro selects between `Bootloader_Run()` (full CRC validation and fallback logic) and `Bootloader_Run_Debug()` (unconditional jump, used during application development to verify the jump mechanism independently of FRAM state).

---

## Repository Structure

All bootloader logic lives inside the `USER CODE` sections of `main.c`. The project is STM32CubeMX-generated; the HAL driver files are unmodified.

```
SpaceSnap-flight-bootloader/
├── Core/
│   ├── Inc/
│   │   └── main.h        # HAL includes, peripheral handles, pin definitions
│   ├── Src/
│   │   └── main.c        # All bootloader logic — see Key Functions below
│   └── Startup/          # Cortex-M3 assembly startup (generated)
├── Drivers/              # STM32F2xx HAL + CMSIS (generated, unmodified)
├── Debug/                # Build artefacts (not tracked by git)
├── Bootloader.ioc        # STM32CubeMX peripheral configuration
├── STM32F217ZGTX_FLASH.ld
├── STM32F217ZGTX_RAM.ld
└── README.md
```

### Key functions (`main.c`)

| Function | Responsibility |
|----------|----------------|
| `Log()` | Blocking UART4 transmit for debug messages; compiled to `((void)0)` when `BOOTLOADER_DEBUG_LOG` is 0 |
| `log_hex()` | Logs a labeled 32-bit value in hex without `sprintf`; avoids pulling the newlib printf chain (~3.3 KB flash saving) |
| `FRAM_Read()` | SPI burst read — WREN not needed; issues opcode `0x03` + 24-bit address MSB first, clocks in `len` bytes |
| `CRC32_Calculate()` | zlib-compatible CRC32 (poly `0xEDB88320`, init/final XOR `0xFFFFFFFF`); matches the application firmware's algorithm exactly |
| `Flash_ErasePages()` | Erases every flash sector overlapping `[start_addr, start_addr + size)`; iterates the `flash_sectors[]` table; IWDG refreshed between sectors |
| `RestoreAppFromFRAM()` | Streams a FRAM image into flash in 256-byte chunks with word-aligned programming and 0xFF-padding of the final partial chunk; IWDG refreshed per chunk; parameterized by source address for copy A/B |
| `JumpToApplication()` | SP/reset-handler sanity checks, `HAL_DeInit()`, SysTick disable, `SCB->VTOR` relocation, MSP set, function-pointer jump |
| `Bootloader_Run()` | Full flight boot logic — flash-first fast path, copy A fallback, copy B fallback, halt |
| `Bootloader_Run_Debug()` | Debug-only — reads copy A header for logging, then unconditionally jumps; never use in flight |

---

## Memory Map

### Internal Flash (1 MB, 12 sectors)

| Sector | Size | Range | Contents |
|--------|------|-------|----------|
| 0 | 16 KB | `0x08000000`–`0x08003FFF` | Bootloader (this firmware) |
| 1 | 16 KB | `0x08004000`–`0x08007FFF` | Application start (`APP_ADDRESS`) |
| 2–3 | 16 KB each | `0x08008000`–`0x0800FFFF` | Application |
| 4 | 64 KB | `0x08010000`–`0x0801FFFF` | Application |
| 5–6 | 128 KB each | `0x08020000`–`0x0805FFFF` | Application (up to `app_size` requires) |
| 7–11 | 128 KB each | `0x08060000`–`0x080FFFFF` | Unused by application at current firmware size |

The application image tested at firmware version 1.1 is 91,880 bytes, fitting comfortably within Sectors 1–5 with significant headroom. Each 128 KB FRAM copy supports images up to 131,060 bytes (128 KB minus the 12-byte header).

### FRAM Backup Region (within shared 2 MB FRAM)

The entire 256 KB window at `0x1C0000–0x1FFFFF` is reserved for firmware backup by both the bootloader and the application firmware. The layout is fixed and **must be kept in sync** between both firmwares — any change to `FIRMWARE_BACKUP_SIZE`, `FIRMWARE_COPY_SIZE`, or `END_OF_FRAM` here must be mirrored in the application's `fram.h`.

**Copy A (`0x1C0000`–`0x1DFFFF`, 128 KB):**

| Absolute Address | Size | Content |
|-----------------|------|---------|
| `0x1C0000` | 4 B | `app_size` — size of the backup image in bytes (uint32, LE) |
| `0x1C0004` | 4 B | `app_crc32` — CRC32 of the backup image (uint32, LE) |
| `0x1C0008` | 4 B | `app_version` — `(VERSION_MAJOR << 16) \| VERSION_MINOR` (uint32, LE) |
| `0x1C000C` | up to 131,060 B | Application image data |

**Copy B (`0x1E0000`–`0x1FFFFF`, 128 KB):**

| Absolute Address | Size | Content |
|-----------------|------|---------|
| `0x1E0000` | 4 B | `app_size` — size of the backup image in bytes (uint32, LE) |
| `0x1E0004` | 4 B | `app_crc32` — CRC32 of the backup image (uint32, LE) |
| `0x1E0008` | 4 B | `app_version` — `(VERSION_MAJOR << 16) \| VERSION_MINOR` (uint32, LE) |
| `0x1E000C` | up to 131,060 B | Application image data |

Both headers are written exclusively by the application's `CMD_BackupFirmware` command (`0x91`) via `SaveFRAM_Unlocked()`. The bootloader treats both regions as **read-only** — it never writes to FRAM under any circumstances.

---

## Boot Decision Logic

`Bootloader_Run()` is called once from `main()` after HAL, GPIO, SPI, and IWDG initialisation. It follows this exact sequence:

```
Read size_a, crc_a from copy A header (FIRMWARE_BACKUP_A_START)
Read size_b, crc_b from copy B header (FIRMWARE_BACKUP_B_START)

Log all four values over UART4

Step 1 — Flash fast path:
    If size_a is valid (non-zero, ≤ FIRMWARE_IMAGE_A_SIZE):
        Calculate CRC32 over flash at APP_ADDRESS for size_a bytes
        If CRC matches crc_a:
            LOG "Flash image matches copy A - booting"
            JumpToApplication()        ← does not return on success
            LOG "Jump failed despite CRC match"
            (fall through to Step 2)

Step 2 — Restore from copy A:
    If size_a is valid:
        RestoreAppFromFRAM(size_a, FIRMWARE_IMAGE_A_START)
        If restore returns HAL_OK:
            Recalculate CRC32 over flash
            If CRC matches crc_a:
                LOG "Restored from copy A - CRC OK"
                a_valid = 1
            Else:
                LOG "Copy A restore CRC mismatch"
    Else:
        LOG "Copy A header invalid"

    If a_valid:
        JumpToApplication()            ← does not return on success
        LOG "Jump failed after restore from copy A"

Step 3 — Restore from copy B:
    LOG "Falling back to copy B"
    If size_b is valid (non-zero, ≤ FIRMWARE_IMAGE_B_SIZE):
        RestoreAppFromFRAM(size_b, FIRMWARE_IMAGE_B_START)
        If restore returns HAL_OK:
            Recalculate CRC32 over flash
            If CRC matches crc_b:
                LOG "Restored from copy B - CRC OK"
                b_valid = 1
            Else:
                LOG "Copy B restore CRC mismatch"
    Else:
        LOG "Copy B header invalid"

    If b_valid:
        JumpToApplication()            ← does not return on success
        LOG "Jump failed after restore from copy B"

Step 4 — Halt:
    LOG "Both firmware backup copies failed - halting"
    Error_Handler()                    ← disables IRQ, infinite loop
```

The step 1 fast path avoids any FRAM restore on the vast majority of boots — the flash image is only invalid if it was corrupted in place by a radiation event or a failed application flash operation. Steps 2 and 3 are the recovery paths, each independently verifying the restored image before jumping rather than trusting the restore succeeded silently.

---

## CRC32 Algorithm

The bootloader uses the same zlib-compatible CRC32 as the application firmware's `CalculateCRC32()` and `CRC32_Update()` functions:

- **Polynomial:** `0xEDB88320` (reflected representation of `0x04C11DB7`)
- **Initial value:** `0xFFFFFFFF`
- **Final XOR:** `0xFFFFFFFF` (complement of accumulator)
- **Bit order:** LSB first (reflected)

The CRC is computed over the raw bytes of the application image starting at `APP_ADDRESS` in flash, for exactly `app_size` bytes as recorded in the backup header. The application's `CMD_BackupFirmware` command computes the same CRC over the same bytes read from flash before writing them to FRAM, so a matching CRC on the bootloader side proves byte-perfect agreement between what was backed up and what is currently in flash.

---

## Watchdog Behavior

The IWDG is started by `MX_IWDG_Init()` with prescaler 64 and reload 4095, giving a nominal timeout window of approximately 5.6–8.2 seconds (the LSI RC oscillator is uncalibrated). Once started, it cannot be stopped in software.

`HAL_IWDG_Refresh()` is called:
- Between every sector erase in `Flash_ErasePages()` — sector erases can each take several hundred milliseconds
- Between every 256-byte chunk in `RestoreAppFromFRAM()` — a 91 KB restore is approximately 360 chunks

A full restore cycle of the largest expected application (≤ 128 KB, spanning up to 6 sectors) is designed to complete well within the IWDG window with margin.

The IWDG continues running after the jump into the application. **The application must service the same IWDG instance independently** — it configures the same prescaler and reload values in its own `MX_IWDG_Init()` and kicks it once per main loop iteration.

---

## Build Configuration

### Flight vs Debug

```c
#define FLIGHT_BUILD        /* comment out to select Bootloader_Run_Debug() */
```

| Build | `BOOT_RUN()` resolves to | Behaviour |
|-------|--------------------------|-----------|
| `FLIGHT_BUILD` defined | `Bootloader_Run()` | Full CRC validation and dual-copy fallback |
| `FLIGHT_BUILD` not defined | `Bootloader_Run_Debug()` | Unconditional jump; reads copy A header for log output only |

### Debug logging

```c
#define BOOTLOADER_DEBUG_LOG    1   /* set to 0 to strip all LOG() calls */
```

When `BOOTLOADER_DEBUG_LOG` is 0, all `LOG()` calls expand to `((void)0)` and the `Log()`/`log_hex()` function bodies are not compiled. This saves flash space at the cost of all UART output. The `log_hex()` function deliberately avoids `sprintf` to prevent pulling the newlib printf chain (~3.3 KB flash overhead).

### Building

1. Open the project in **STM32CubeIDE**.
2. Confirm `FLIGHT_BUILD` is defined and `BOOTLOADER_DEBUG_LOG` is set appropriately.
3. Build with **Project → Build Project** (`Ctrl+B`).
4. Flash Sector 0 only via ST-LINK. The application occupies Sectors 1+; flashing the bootloader must not overwrite the application, and vice versa. Use separate flash configurations or confirm linker script address ranges before flashing.

The linker script places all bootloader code in `[0x08000000, 0x08004000)` — the build will fail with a linker error if the compiled binary exceeds 16 KB, enforcing the sector boundary constraint at build time.

---

## Known Limitations / Pending Work

- **Self-healing copy repair** — after a successful fallback boot from copy B (copy A invalid), the bootloader does not repair copy A from copy B's known-good image. The system continues running on a single remaining good copy until the ground re-issues `CMD_BackupFirmware`. The application's `fw_backup_mismatch` telemetry flag (set by `CMD_GetStatus` after comparing both headers) provides the ground with the visibility needed to trigger a manual repair. Autonomous repair in the bootloader is deferred; see application README Pending Work.
- **FRAM read error handling** — `FRAM_Read()` does not check the `HAL_StatusTypeDef` return values of `HAL_SPI_Transmit()`/`HAL_SPI_Receive()`. A SPI bus fault is not currently distinguishable from valid-but-wrong data at the read level; the downstream CRC check catches the symptom regardless of cause, but a HAL-level error is not separately logged or counted.
- **Copy B not validated against copy A's CRC** — the bootloader uses `crc_b` (from copy B's own header) to validate a copy B restore, not `crc_a`. If both headers were independently corrupted to the same wrong value (extremely unlikely), the bootloader would accept a corrupt image. In practice, a per-copy header CRC check (CRC of the header itself, not the image) would close this gap but adds complexity not currently justified given the probability.
