# SD Card Driver

## 1. Introduction

This is a SD Card driver. It is based on the [tutorial](https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/) from Dr. Hammond Pearce. The SD Card driver is implemented using the [FatFs](http://elm-chan.org/fsw/ff/00index_e.html) library while communication with the SD Card is done using the SPI protocol.

It has been bonified with helper functions and configurations that allow:

- Easier writing to the SD Card
- Easier flushing of the SD Card
- Periodic flushing of the SD Card with the interrupt of timer 14

## 2. Setup and Connections

The SPI protocol for the SD Card is connected to the IEPE V7 as follows (we will be using SPI1):

SPI1_NSS: PA4 (slave select line for the master to decide which device to send data to)
SPI1_SCK: PA5 (clock signal line)
SPI1_MISO: PA6 (master input/slave output)
SPI1_MOSI: PA7 (master input/ slave output)

Because we have separate MISO MOSI lines, we can send and recieve data simeoultaneously in full dpulex mode. Note that CS/SS line is active when low (this will be useful to know when we want to check if our sd card is mounted correctly).
CPHA determines if we sample on first or second clock edge (0 is first edge, both are rising/falling edge), CPOL determines if clock idles on low or high (0=low).
SPI protocal is not standardized, most devices support mode 0 (which means that CPHA = CPOL = 0), but we have to look at the slave’s data sheet to make sure.

Note that we are using the STM32F446RE Nucleo board, so the connections are made according to this board. If you are using another board, you should check the pinout and make the connections accordingly.

### 2.1. SD Card

The SD card should be formatted to MS-DOS (FAT32) format for its file system to be compatible with the current driver implementation.

Please note that according to the official [FatFs documentation](http://elm-chan.org/fsw/ff/doc/appnote.html), the following are the limitations of the FAT file system:

- Number of open files: Unlimited. (depends on available memory)
- Number of volumes: Up to 10.
- Sector size: 512, 1024, 2048 and 4096 bytes.
- Minimum volume size: 128 sectors.
- Maximum volume size: 2^32 - 1 sectors in 32-bit LBA.
- Maximum file size: 2^32 - 1 bytes on FAT volume.
- Cluster size: Upto 128 sectors on FAT volume.
- read/write performance will be improved by taking effort to sector align.

### 2.2. IOC

#### 2.2.1. Middleware

Finally, as we’re going to be using the SD card with the FAT file system, scroll down in the device categories to Middleware, and expand this, then enable FATFS as User-defined. You may leave all other parameters as their defaults.

#### 2.2.2. Clock and Timer Configurations

You may need to modify them according to your desired clock speeds. For us, HCLK = 72MHz.
We will not be using a timer to write to our sd card, we will interrupt it using the accelormeter and telemetry statuses inside of our megaloop.

#### 2.2.3. Connectivity

Select SPI1 under connectivity and mode Full-Duplex Master with Hardware NSS Output Signal (SPI peripheral automatically controls the CS line for us, setting it LOW when we start sending data and back to HIGH when we want the slave to ignore the bus). Set data size to 8 bits, prescaler at 256 (the Baud Rate should be approximately `280 KBits/s`)

### 2.3. Files

Add the files `user_diskio_spi.c` and `user_diskio_spi.h` found under `FATFS/Target` into your own `FATFS/Target`. Also add `sdcard.h` and `sdcard.c`.

#### 2.3.1. user_diskio.c

Add lines according to:

```C
/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "user_diskio_spi.h"
```

```C
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
  BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  return USER_SPI_initialize(pdrv); //ADD THIS LINE
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
  BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  return USER_SPI_status(pdrv); //ADD THIS LINE
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
  BYTE pdrv,      /* Physical drive nmuber to identify the drive */
  BYTE *buff,     /* Data buffer to store read data */
  DWORD sector,   /* Sector address in LBA */
  UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  return USER_SPI_read(pdrv, buff, sector, count); //ADD THIS LINE
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
  BYTE pdrv,          /* Physical drive nmuber to identify the drive */
  const BYTE *buff,   /* Data to be written */
  DWORD sector,       /* Sector address in LBA */
  UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
  return USER_SPI_write(pdrv, buff, sector, count); //ADD THIS LINE
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
  BYTE pdrv,      /* Physical drive nmuber (0..) */
  BYTE cmd,       /* Control code */
  void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  return USER_SPI_ioctl(pdrv, cmd, buff); //ADD THIS LINE
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
```

#### 2.3.2. user_diskio_spi.c

Modify the lines defining `FCLK_SLOW` and `FCLK_FAST`. You can use the baud rate prescaler in the ioc shown in section _2.2.3._ to calculate the needed values.

Note that you only need to modify the second `SPI_BAUDRATEPRESCALER` values since the first one is set to 256 to serve as mask to clear the prescaler bits.

**Example:**

for a HCLK of 72 MHz, the baud rate prescaler is set to 128 for slow and 8 for fast.

```C
#define FCLK_SLOW() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_256); } /* Set SCLK = slow, approx 280 KBits/s*/
#define FCLK_FAST() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_16); } /* Set SCLK = fast, approx 4.5 MBits/s */
```

#### 2.3.3. main.h

Add lines according to:

```C
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */
```

## 3. Description

Inside `sdcard.c`, you can find the helper functions:

```c
bool write_packet(PL_SDCard_Handler *sd_card, const char *prefix, size_t size_prefix, const void *data, size_t size_data);
```

This function passes our sd card struct, the prefix (telemetry or accelerometer data), the size of the prefix (because we want to write it into the file to parse it later), and the data and size of data to write into the file.

This function will automatically flush the data once the number of bytes written to the file exceeds `SD_FLUSH_BYTES` so that we conserve time by not flushing after every packet + we don't keep to much data in memory in case of data loss.

Functions called in `main.c`:

```c
bool PL_SDCard_Init(PL_SDCard_Handler *sd_card);
bool PL_SDCard_Open(PL_SDCard_Handler *sd_card);
```

## 4. Miscellaneous Edge Cases Resolved During Development

### 4.1. Mounting the SD Card

The `FR_NOT_READY (3)` error during the mounting of the SD card can be caused by various reasons. It is not very descriptive, so it is hard to know what is actually causing the error. The following are some of the reasons that can cause the error:

- The SD card is not properly connected to the SPI pins.
- The SD card is not properly formatted.
- The baud rate prescaler is not properly set.
- The GPIO Pull-up/Pull-down settings of the SPI pins might need to be adjusted ([see here](https://github.com/kiwih/cubeide-sd-card/issues/2)).
