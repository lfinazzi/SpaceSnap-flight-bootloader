/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ------------------------------------------------------------------------
 * STM32F217ZG: 1 MB flash, 12 sectors (standard F2/F4 layout):
 *   Sector 0:  16 KB  0x08000000 - 0x08003FFF
 *   Sector 1:  16 KB  0x08004000 - 0x08007FFF   <-- APP_ADDRESS starts here
 *   Sector 2:  16 KB  0x08008000 - 0x0800BFFF
 *   Sector 3:  16 KB  0x0800C000 - 0x0800FFFF
 *   Sector 4:  64 KB  0x08010000 - 0x0801FFFF
 *   Sector 5: 128 KB  0x08020000 - 0x0803FFFF
 *   Sector 6: 128 KB  0x08040000 - 0x0805FFFF
 *   Sector 7: 128 KB  0x08060000 - 0x0807FFFF
 *   Sector 8: 128 KB  0x08080000 - 0x0809FFFF
 *   Sector 9: 128 KB  0x080A0000 - 0x080BFFFF
 *   Sector 10:128 KB  0x080C0000 - 0x080DFFFF
 *   Sector 11:128 KB  0x080E0000 - 0x080FFFFF
 *
 * APP_ADDRESS falls exactly on the Sector 1 boundary, so no alignment
 * issues. FIRMWARE_BACKUP_SIZE (256 KB) means the app region can span
 * up to Sectors 1-6 depending on actual app_size.
 * ---------------------------------------------------------------------- */
typedef struct {
    uint32_t   base;
    uint32_t   size;
    uint32_t   hal_id;
} flash_sector_t;

// flash sector architecture
static const flash_sector_t flash_sectors[] = {
    { 0x08000000, 16  * 1024,  FLASH_SECTOR_0  },
    { 0x08004000, 16  * 1024,  FLASH_SECTOR_1  },
    { 0x08008000, 16  * 1024,  FLASH_SECTOR_2  },
    { 0x0800C000, 16  * 1024,  FLASH_SECTOR_3  },
    { 0x08010000, 64  * 1024,  FLASH_SECTOR_4  },
    { 0x08020000, 128 * 1024,  FLASH_SECTOR_5  },
    { 0x08040000, 128 * 1024,  FLASH_SECTOR_6  },
    { 0x08060000, 128 * 1024,  FLASH_SECTOR_7  },
    { 0x08080000, 128 * 1024,  FLASH_SECTOR_8  },
    { 0x080A0000, 128 * 1024,  FLASH_SECTOR_9  },
    { 0x080C0000, 128 * 1024,  FLASH_SECTOR_10 },
    { 0x080E0000, 128 * 1024,  FLASH_SECTOR_11 },
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* ------------------------------------------------------------------------
 * Debug logging over UART4 - strip for flight builds to save flash space
 * (the whole bootloader must fit in a single 16 KB sector). Comment out
 * the #define below to compile Log() calls down to nothing.
 * ---------------------------------------------------------------------- */
#define BOOTLOADER_DEBUG_LOG   		1

#if BOOTLOADER_DEBUG_LOG
	#define LOG(msg)  				Log(msg)
	#define LOG_UART_TIMEOUT        (100U)
#else
	#define LOG(msg)  ((void)0)
#endif 	/* BOOTLOADER_DEBUG_LOG */

/* ------------------------------------------------------------------------
 * Debug test that the bootloader works without checking CRC,
 * change for flight for release.
 * ---------------------------------------------------------------------- */
#define FLIGHT_BUILD					0

#if FLIGHT_BUILD
	#define BOOT_RUN()					Bootloader_Run()
#else
	#define BOOT_RUN()					Bootloader_Run_Debug()
#endif	/* FLIGHT_BUILD */

/* ------------------------------------------------------------------------
 * Memory map
 * ---------------------------------------------------------------------- */
#define APP_ADDRESS                    (0x08004000UL)

#define FIRMWARE_BACKUP_SIZE           (0x40000UL)    /* 256 kB for FW backup image */
#define END_OF_FRAM                    (0x200000UL)   /* 2 MB */

// structure: app size (4B), app CRC32 (4B), firmware image (rest of backup size)
#define FIRMWARE_BACKUP_START          ((END_OF_FRAM) - (FIRMWARE_BACKUP_SIZE))

#define FRAM_ADDR_APP_SIZE             (FIRMWARE_BACKUP_START)            /* uint32_t, 4 bytes */
#define FRAM_ADDR_APP_CRC              ((FIRMWARE_BACKUP_START) + (4U))   /* uint32_t, 4 bytes */
#define FRAM_ADDR_APP_VERSION          ((FIRMWARE_BACKUP_START) + (8U))   /* uint32_t, 4 bytes */

#define FIRMWARE_IMAGE_START           ((FIRMWARE_BACKUP_START) + (12U))
#define FIRMWARE_IMAGE_SIZE			   ((FIRMWARE_BACKUP_SIZE) - (12U))

// Number of flash sectors in chip
#define NUM_FLASH_SECTORS (sizeof(flash_sectors) / sizeof(flash_sectors[0]))

// Internal MCU SRAM valid region
#define APP_SRAM_BASE   0x20000000UL
#define APP_SRAM_SIZE   0x00020000UL   /* 128 KB */
#define APP_SRAM_END    (APP_SRAM_BASE + APP_SRAM_SIZE)

/* ------------------------------------------------------------------------
 * FRAM read/write
 * ---------------------------------------------------------------------- */
#define FRAM_CMD_READ                  (0x03)
#define FRAM_CMD_WRITE                 (0x02)
#define FRAM_CMD_WREN                  (0x06)

// FRAM chip-select - PB12
#define FRAM_CS_LOW()   HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET)
#define FRAM_CS_HIGH()  HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_SET)

// Size of chunks on fw recovery from FRAM
#define FLASH_PROGRAM_CHUNK_SIZE       (256U)   /* bytes read from FRAM per loop iteration */

// FLASH_PROGRAM_CHUNK_SIZE must be a multiple of 4 for word-aligned flash programming
typedef char FLASH_PROGRAM_CHUNK_SIZE_must_be_multiple_of_4
    [(FLASH_PROGRAM_CHUNK_SIZE % 4 == 0) ? 1 : -1];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if BOOTLOADER_DEBUG_LOG
	/********************************************************************************
	 * @brief  Transmits a timestamped message over the debug UART (huart4).
	 *
	 * @note   Transmission is blocking with LOG_UART_TIMEOUT ms timeout per call.
	 *         Uses strlen() to determine message length -- the input string must
	 *         be null-terminated. Updates the global timestamp, total_seconds,
	 *         hours, minutes, seconds, and timestamp_string variables as a side
	 *         effect.
	 *
	 * @param  message Null-terminated string to transmit.
	 ********************************************************************************/
	void Log(char *message)
	{
	    HAL_UART_Transmit(&huart4, (uint8_t *) message, strlen(message), LOG_UART_TIMEOUT);
	    return;
	}


	/********************************************************************************
	 * @brief  Logs a labeled 32-bit value in hexadecimal over the debug UART.
	 *
	 * @note   Prints prefix followed immediately by the value formatted as
	 *         "0xXXXXXXXX\r\n". Avoids sprintf and the newlib printf chain
	 *         entirely, saving ~3.3 KB of flash (sprintf, malloc, __udivmoddi4).
	 *         Output is always 10 characters for the hex value plus CRLF.
	 *
	 * @param  prefix   Null-terminated label string printed before the value.
	 * @param  val      32-bit value to print.
	 *
	 * @retval None
	 ********************************************************************************/
	void log_hex(char *prefix, uint32_t val)
	{
	    char buf[16];
	    const char hex[] = "0123456789ABCDEF";
	    buf[0]='0'; buf[1]='x';
	    for (int i = 0; i < 8; i++)
	        buf[2+i] = hex[(val >> (28 - i*4)) & 0xF];
	    buf[10]='\r'; buf[11]='\n'; buf[12]='\0';
	    LOG(prefix);
	    LOG(buf);
	}
#endif 	/* BOOTLOADER_DEBUG_LOG */


/********************************************************************************
 * @brief  Reads a sequence of bytes from the FRAM starting at the specified
 *         address.
 *
 * @note   Transmits the READ opcode followed by the 24-bit address MSB first,
 *         then clocks in len bytes of data. CS is asserted for the full
 *         transaction and deasserted on completion.
 *
 * @param  addr   24-bit memory address to read from.
 * @param  buf    Pointer to the buffer where read data will be stored.
 * @param  len    Number of bytes to read.
 *
 * @retval None
 ********************************************************************************/
void FRAM_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint8_t cmd[4] = {
		FRAM_CMD_READ,
		(uint8_t)((addr >> 16) & 0xFF),
		(uint8_t)((addr >> 8) & 0xFF),
		(uint8_t)(addr & 0xFF)
	};

	FRAM_CS_LOW();
	HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
	FRAM_CS_HIGH();
}


/********************************************************************************
 * @brief  Computes the CRC32 checksum of a data buffer.
 *
 * @note   Uses the zlib-compatible CRC32 algorithm: polynomial 0xEDB88320
 *         (reflected), initial value 0xFFFFFFFF, and final XOR 0xFFFFFFFF.
 *         Must match whatever algorithm originally computed stored_crc
 *         in FRAM.
 *
 * @param  data   Pointer to the buffer to checksum.
 * @param  len    Number of bytes in the buffer.
 *
 * @retval uint32_t   Computed CRC32 value.
 ********************************************************************************/
uint32_t CRC32_Calculate(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
        }
    }

    return ~crc;
}


/********************************************************************************
 * @brief  Erases all flash sectors that overlap a given address range.
 *
 * @note   Iterates the flash_sectors[] table and erases (one sector at a
 *         time) every sector whose range intersects [start_addr, start_addr
 *         + size). Aborts and returns the HAL error code on the first
 *         failed erase; sectors after the failure are left untouched.
 *         Assumes 2.7V-3.6V Vdd (FLASH_VOLTAGE_RANGE_3).
 *
 * @pre Flash must be unlocked via HAL_FLASH_Unlock() before calling.
 *
 * @param  start_addr   First address of the range to erase.
 * @param  size         Size in bytes of the range to erase.
 *
 * @retval HAL_StatusTypeDef   HAL_OK if all overlapping sectors erased
 *                             successfully, otherwise the HAL error from
 *                             the failing HAL_FLASHEx_Erase call.
 ********************************************************************************/
HAL_StatusTypeDef Flash_ErasePages(uint32_t start_addr, uint32_t size)
{
    uint32_t end_addr = start_addr + size;

    for (uint32_t i = 0; i < NUM_FLASH_SECTORS; i++) {
        uint32_t sec_start = flash_sectors[i].base;
        uint32_t sec_end   = sec_start + flash_sectors[i].size;

        /* Does this sector overlap [start_addr, end_addr)? */
        if (sec_start < end_addr && sec_end > start_addr) {
            FLASH_EraseInitTypeDef erase = {0};
            uint32_t sector_error = 0;

            erase.TypeErase   = FLASH_TYPEERASE_SECTORS;

            // RANGE_3 assumes 2.7V-3.6V Vdd
            erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            erase.Sector       = flash_sectors[i].hal_id;
            erase.NbSectors    = 1;

            HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
            if (status != HAL_OK) {
                return status;
            }
        }
        HAL_IWDG_Refresh(&hiwdg);   /* service watchdog between sector erases */
    }

    return HAL_OK;
}


/********************************************************************************
 * @brief  Restores the application image from FRAM backup into internal
 *         flash and re-arms it for boot.
 *
 * @note   Validates app_size against the FRAM backup region, erases the
 *         flash sectors covering [APP_ADDRESS, APP_ADDRESS + app_size),
 *         then streams the image from FRAM in FLASH_PROGRAM_CHUNK_SIZE
 *         chunks, programming each chunk to flash one word at a time.
 *         The final partial chunk is padded to a 4-byte boundary with
 *         0xFF before programming. Flash is unlocked for the duration
 *         of the operation and locked again before every return path
 *         (success or failure).
 *
 * @param  app_size   Size in bytes of the application image to restore,
 *                     as read from the FRAM backup header.
 *
 * @retval HAL_StatusTypeDef   HAL_OK on success. HAL_ERROR if app_size is
 *                             zero or exceeds the FRAM backup capacity.
 *                             Otherwise the HAL error from the failing
 *                             erase or program operation.
 ********************************************************************************/
HAL_StatusTypeDef RestoreAppFromFRAM(uint32_t app_size)
{
    HAL_StatusTypeDef status;
    uint8_t chunk[FLASH_PROGRAM_CHUNK_SIZE];
    uint32_t fram_offset = FIRMWARE_IMAGE_START;
    uint32_t flash_addr  = APP_ADDRESS;
    uint32_t remaining   = app_size;

    if (app_size == 0 || app_size > FIRMWARE_IMAGE_SIZE) {
        LOG("Restore aborted: invalid app_size from FRAM\r\n");
        return HAL_ERROR;   /* sanity check - corrupt/garbage size field */
    }

    LOG("Restoring app...\r\n");

    HAL_FLASH_Unlock();

    status = Flash_ErasePages(APP_ADDRESS, app_size);
    if (status != HAL_OK) {
        LOG("Flash erase failed\r\n");
        HAL_FLASH_Lock();
        return status;
    }

    LOG("Erase complete, programming flash...\r\n");

    while (remaining > 0) {
        uint32_t n = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;

        FRAM_Read(fram_offset, chunk, n);

        /* Pad final partial chunk to a 4-byte boundary with 0xFF
         * (erased-flash value) so the word program below never reads
         * past the buffer. */
        uint32_t n_words = (n + 3) / 4;
        if (n % 4 != 0) {
            memset(&chunk[n], 0xFF, (n_words * 4) - n);
        }

        for (uint32_t i = 0; i < n_words * 4; i += 4) {
            uint32_t word;
            memcpy(&word, &chunk[i], 4);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + i, word);
            if (status != HAL_OK) {
                LOG("Flash program failed.\r\n");
                HAL_FLASH_Lock();
                return status;
            }
        }

        fram_offset += n;
        flash_addr   += n;
        remaining     -= n;
        HAL_IWDG_Refresh(&hiwdg);   /* service watchdog between chunks */
    }

    HAL_FLASH_Lock();
    LOG("Restore complete\r\n");
    return HAL_OK;
}


/********************************************************************************
 * @brief  Relocates the vector table and transfers execution to the
 *         application image in flash.
 *
 * @note   Reads the initial stack pointer and reset handler address from
 *         the application's vector table at APP_ADDRESS, validates both
 *         against basic sanity bounds, disables interrupts, relocates
 *         SCB->VTOR to APP_ADDRESS, sets the main stack pointer, then
 *         branches to the application's reset handler. Does not return
 *         on success. Returns early (without jumping) if the vector
 *         table looks invalid/erased, e.g. no application flashed.
 *
 * @param  None
 *
 * @retval None   Function does not return if the jump succeeds. If it
 *                returns, the jump was aborted due to a failed sanity
 *                check and the caller must decide how to proceed
 *                (e.g. stay in bootloader, signal an error).
 ********************************************************************************/
typedef void (*pFunction)(void);
HAL_StatusTypeDef JumpToApplication(void)
{
    uint32_t appStack = *(__IO uint32_t*)(APP_ADDRESS);
    uint32_t appReset  = *(__IO uint32_t*)(APP_ADDRESS + 4);

    log_hex("JumpToApp: ", appStack);

    /* Sanity check: initial SP must point somewhere inside SRAM.
     * Catches the common case of erased/unprogrammed flash (0xFFFFFFFF). */
    if (appStack < APP_SRAM_BASE || appStack > APP_SRAM_END) {
        LOG("Jump aborted: invalid app stack pointer\r\n");
        return HAL_ERROR;
    }

    /* Sanity check: reset handler address must have the Thumb bit set
     * (odd address) - Cortex-M only executes Thumb code. */
    if ((appReset & 0x1) == 0) {
        LOG("Jump aborted: invalid app reset handler address\r\n");
        return HAL_ERROR;
    }


    /* Reset peripherals the bootloader configured (FRAM SPI, any clocks,
	 * GPIO, etc.) back to their power-on state so the app starts from a
	 * known baseline, same as it would on a real POR. */
	HAL_DeInit();

	/* Disable SysTick before handing off - the app's own SystemInit/
	 * startup code will reconfigure it. Leaving the bootloader's SysTick
	 * running with interrupts re-enabled later could fire into the app
	 * before its vector table is fully in effect. */
	SysTick->CTRL = 0;
	SysTick->VAL  = 0;

	__disable_irq();
	SCB->VTOR = APP_ADDRESS;
	__set_MSP(appStack);
	pFunction jump = (pFunction)appReset;
	jump();

	/* never reached */

	/* Truly unreachable: jump() above never returns on success, and we
	 * already returned HAL_ERROR above for any case that would prevent
	 * the jump. This line exists only to satisfy the compiler's
	 * expectation of a return path. */
	return HAL_ERROR;
}


/********************************************************************************
 * @brief  Top-level boot decision logic: validates the application image
 *         in flash and either boots it, restores it from FRAM backup,
 *         or halts.
 *
 * @note   Reads app_size and stored_crc from the FRAM backup header and
 *         sanity-checks app_size before using it. Computes the CRC32 of
 *         the current flash image and compares against stored_crc. On
 *         match, attempts to jump to the application. On mismatch (or
 *         if the jump itself fails its own sanity checks), assumes the
 *         flash image is corrupted and attempts to restore it from the
 *         FRAM backup via RestoreAppFromFRAM(), then re-validates the
 *         CRC and attempts the jump again. If no valid, jumpable image
 *         can be obtained, calls Error_Handler() and does not return.
 *         Must be called from main() after HAL and SPI (FRAM)
 *         peripheral init.
 *
 * @param  None
 *
 * @retval None   Does not return if a valid image is successfully
 *                jumped to. Otherwise falls through to Error_Handler(),
 *                which is expected to not return either (e.g. infinite
 *                loop or reset).
 ********************************************************************************/
void Bootloader_Run(void)
{
    uint32_t stored_crc  = 0;
    uint32_t app_size    = 0;
    uint32_t app_version = 0;
    uint32_t calculated_crc;

    LOG("FLIGHT Bootloader started\r\n");

    FRAM_Read(FRAM_ADDR_APP_SIZE, (uint8_t*)&app_size, sizeof(app_size));
    FRAM_Read(FRAM_ADDR_APP_CRC, (uint8_t*)&stored_crc, sizeof(stored_crc));
    FRAM_Read(FRAM_ADDR_APP_VERSION, (uint8_t*)&app_version, sizeof(app_version));

    log_hex("FRAM: stored_crc= ", stored_crc);
    log_hex("FRAM: stored_size= ", app_size);
    log_hex("FRAM: stored_version= ", app_version);

    /* Sanity check app_size before using it as a CRC/read length -
     * FRAM could be blank or corrupted (e.g. first boot), in which case
     * app_size may be garbage or absurdly large. */
    if (app_size == 0 || app_size > FIRMWARE_IMAGE_SIZE) {
        LOG("Invalid app_size from FRAM - skipping flash CRC check\r\n");
    } else {
        calculated_crc = CRC32_Calculate((uint8_t*)APP_ADDRESS, app_size);

        log_hex("Calculated CRC of flash image: ", calculated_crc);

        if (calculated_crc == stored_crc) {
            LOG("CRC match - app image OK\r\n");
            JumpToApplication();

            // If it doesn't jump, there was an error
            LOG("Jump failed despite CRC match - vector table invalid\r\n");
        }
        else {
            LOG("CRC mismatch - flash image corrupted, restoring from FRAM\r\n");
        }
    }

    /* Flash image was missing, invalid-sized, CRC-mismatched, or failed
     * its own jump sanity check - attempt restore from FRAM backup. */
    if (app_size != 0 && app_size <= FIRMWARE_IMAGE_SIZE) {
        if (RestoreAppFromFRAM(app_size) == HAL_OK) {
            calculated_crc = CRC32_Calculate((uint8_t*)APP_ADDRESS, app_size);
            if (calculated_crc == stored_crc) {
                LOG("Restored image CRC OK\r\n");
                JumpToApplication();

                // If it doesn't jump, there was an error
                LOG("Jump failed after restore - vector table invalid\r\n");
            } else {
                LOG("Restored image CRC still mismatched\r\n");
            }
        }
    }

    /* Flash image bad, restore attempt bad, or jump itself failed its
     * sanity checks - do not jump. */
    Error_Handler();
}


/********************************************************************************
 * @brief  Top-level bootloader. Always goes to application. Used for debug!
 *
 * @param  None
 *
 * @retval None
 ********************************************************************************/
void Bootloader_Run_Debug(void)
{
	uint32_t stored_crc  = 0;
	uint32_t app_size    = 0;
	uint32_t app_version = 0;
    LOG("Debug Bootloader started\r\n");

    FRAM_Read(FRAM_ADDR_APP_SIZE, (uint8_t*)&app_size, sizeof(app_size));
	FRAM_Read(FRAM_ADDR_APP_CRC, (uint8_t*)&stored_crc, sizeof(stored_crc));
	FRAM_Read(FRAM_ADDR_APP_VERSION, (uint8_t*)&app_version, sizeof(app_version));

	log_hex("FRAM: stored_crc= ", stored_crc);
	log_hex("FRAM: stored_size= ", app_size);
	log_hex("FRAM: stored_version= ", app_version);

    // Always jumps to app. Function is used to check jump is implemented ok
    JumpToApplication();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Main bootloader logic
  BOOT_RUN();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_N_GPIO_Port, CS_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_N_Pin */
  GPIO_InitStruct.Pin = CS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_N_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  LOG("FATAL: flash corrupt and FRAM restore failed - halting\r\n");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
