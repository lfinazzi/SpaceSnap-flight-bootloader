# UNSAM SpaceSnap (USS) — Bootloader

Bootloader for the **UNSAM SpaceSnap** imaging payload — runs on every reset of the payload MCU aboard **UNSAMSat-01**, validates the application image in internal flash, and restores it from a FRAM-backed copy if validation fails before handing off execution to the application.

---

## Overview

| Item | Details |
|------|---------|
| **MCU** | STM32F217ZGTX (Arm Cortex-M3, 120 MHz) — same part as the application firmware |
| **Image validation** | CRC32 (zlib-compatible) of the application region in flash against a checksum stored in FRAM |
| **Recovery source** | FRAM backup region — `CY15B108QSN`, 256 KB reserved at FRAM offset `0x1C0000` |
| **Application entry point** | `0x08004000` (Sector 1) |
| **Bootloader footprint** | Sector 0 only (16 KB) |
| **Debug interface** | UART4, 115200 baud (compiled out via `BOOTLOADER_DEBUG_LOG` for flight builds) |
| **Watchdog** | Independent Watchdog (IWDG), shared with and inherited by the application after jump |
| **Build toolchain** | STM32CubeIDE (Eclipse CDT / GCC ARM) |

---

## Features

- **CRC32 image validation** — computes a zlib-compatible CRC32 over the application region in flash and compares it against a checksum maintained in FRAM before ever jumping to it.
- **FRAM-backed recovery** — on a CRC mismatch, erases and reprograms the application region in flash from a backup image stored in FRAM, then re-validates before retrying the jump.
- **Vector-table sanity checks** — before jumping, confirms the application's initial stack pointer falls inside valid SRAM and its reset handler address has the Thumb bit set, catching blank/erased flash without relying on a successful CRC check alone.
- **Watchdog-aware flash operations** — `HAL_IWDG_Refresh()` is called between sector erases and between program chunks so multi-second flash erase operations don't trip the IWDG mid-restore.
- **Fail-safe halt** — if neither the existing flash image nor the FRAM-restored image validates, the bootloader halts in `Error_Handler()` rather than jumping into unknown code.

---

## Repository Structure

This is a single-file STM32CubeMX-generated project; all bootloader logic currently lives inside the `USER CODE` sections of `main.c` rather than being split into separate modules (unlike the application firmware).

```
Bootloader/
├── Core/
│   ├── Inc/          # Application header files
│   ├── Src/
│   │   └── main.c    # HAL init, boot decision logic, all bootloader functions
│   └── Startup/      # Cortex-M3 assembly startup
├── Drivers/          # STM32F2xx HAL + CMSIS (generated)
├── Debug/            # Build artefacts (not tracked by git)
├── Bootloader.ioc     # STM32CubeMX peripheral configuration
├── STM32F217ZGTX_FLASH.ld
├── STM32F217ZGTX_RAM.ld
└── README.md
```

### Key functions (`main.c`)

| Function | Responsibility |
|----------|----------------|
| `Log` | Timestamped debug logging over UART4; compiled to a no-op when `BOOTLOADER_DEBUG_LOG` is 0 |
| `FRAM_Read` | Raw SPI FRAM read — opcode `0x03` + 24-bit address, MSB first |
| `CRC32_Calculate` | zlib-compatible CRC32 over an arbitrary byte range (flash or FRAM) |
| `Flash_ErasePages` | Erases every flash sector overlapping a given `[start, start+size)` range |
| `RestoreAppFromFRAM` | Streams the FRAM-backed image into flash in chunks, word-programming each chunk |
| `JumpToApplication` | Vector-table sanity checks, peripheral/SysTick teardown, VTOR relocation, and jump into the app |
| `Bootloader_Run` | Top-level decision logic: validate → jump, or restore → re-validate → jump, or halt |

---

## Memory Map

### Internal Flash (1 MB, 12 sectors)

| Sector | Size | Range | Contents |
|--------|------|-------|----------|
| 0 | 16 KB | `0x08000000`–`0x08003FFF` | Bootloader |
| 1 | 16 KB | `0x08004000`–`0x08007FFF` | Application start (`APP_ADDRESS`) |
| 2–3 | 16 KB each | `0x08008000`–`0x0800FFFF` | Application |
| 4 | 64 KB | `0x08010000`–`0x0801FFFF` | Application |
| 5–11 | 128 KB each | `0x08020000`–`0x080FFFFF` | Application (only as many as `app_size` requires) |

The application region can span up to ~256 KB (Sectors 1 through part of 6), matching `FIRMWARE_BACKUP_SIZE` below.

### FRAM Backup Region (offset `0x1C0000`, within the shared 2 MB FRAM)

| Offset (from `0x1C0000`) | Size | Content |
|---------------------------|------|---------|
| `+0x00` | 4 B | `app_size` (uint32, LE) |
| `+0x04` | 4 B | `stored_crc` — CRC32 of the application image |
| `+0x08` | up to 256 KB − 8 B | Application image backup |

This region and offset are shared by contract with the application firmware's own FRAM map, where the same 256 KB window is reserved at the same address. Any change to `FIRMWARE_BACKUP_SIZE` or `END_OF_FRAM` here must be mirrored on the application side, or the two firmwares will silently disagree about where the backup image lives.

---

## Boot Decision Logic

`Bootloader_Run()`, called once from `main()` after HAL/SPI init, follows this sequence:

1. Read `app_size` and `stored_crc` from the FRAM backup header.
2. Sanity-check `app_size` against the FRAM backup capacity before using it as a length for anything.
3. Compute the CRC32 of the current flash image at `APP_ADDRESS` and compare against `stored_crc`.
4. On match, attempt `JumpToApplication()`.
5. On mismatch (or an invalid `app_size`, or a jump that fails its own vector-table sanity checks), call `RestoreAppFromFRAM()` to erase and reprogram the application region from the FRAM backup, then repeat the CRC check and jump attempt once.
6. If no valid, jumpable image can be obtained by this point, call `Error_Handler()`, which disables interrupts and halts.

The FRAM backup header is treated as read-only by this firmware — it is written externally (e.g. a ground-side flashing tool) before deployment and is not modified by the bootloader itself.

---

## Watchdog Behavior

The IWDG is configured for a timeout window of roughly 5.6–8.2 seconds (varying with LSI's uncalibrated RC frequency) and starts automatically during `MX_IWDG_Init()`. Because flash sector erase can take on the order of a second or more per sector, `HAL_IWDG_Refresh()` is called between sector erases in `Flash_ErasePages` and between chunks in `RestoreAppFromFRAM` so a full restore cycle doesn't trip the watchdog mid-operation.

The IWDG cannot be disabled in software once started, so it continues running across the jump into the application — **the application must service the same watchdog instance independently**, or it will reset shortly after a successful boot.

---

## Building

1. Open the project in **STM32CubeIDE**.
2. Select the **Debug** build configuration.
3. Build with **Project → Build Project** (or `Ctrl+B`).
4. Flash via ST-LINK using **Run → Debug** or the flash programmer.

The `Bootloader.ioc` file can be opened in **STM32CubeMX** to regenerate HAL drivers or modify peripheral configuration.

---

## Known Limitations / Pending Work

- **FRAM read error handling** — `FRAM_Read` does not check the return status of `HAL_SPI_Transmit`/`HAL_SPI_Receive`; a SPI bus fault is not currently distinguishable from valid-but-wrong data and relies on the downstream CRC check to catch it.
- **Single backup slot** — only one firmware backup image is retained in FRAM; a corrupted backup itself cannot be recovered from on-device.
- **No on-device backup write path** — the FRAM header (`app_size` + CRC32) and image are expected to be written externally before launch; this firmware only ever reads that region.