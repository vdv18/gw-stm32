#include "microsd.h"
#include "diskio.h"

static volatile DSTATUS Stat = STA_NOINIT;
#define _USE_WRITE      1
#define _USE_IOCTL      1

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
static DSTATUS microsd_disk_initialize(BYTE lun);
static DSTATUS microsd_disk_status(BYTE lun);
static DRESULT microsd_disk_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
static DRESULT microsd_disk_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
static DRESULT microsd_disk_ioctl(BYTE lun, BYTE cmd, void *buff);

static DSTATUS ram_disk_initialize(BYTE lun){return STA_NOINIT;};
static DSTATUS ram_disk_status(BYTE lun){return STA_NOINIT;};
static DRESULT ram_disk_read(BYTE lun, BYTE *buff, DWORD sector, UINT count){return STA_NOINIT;};
static DRESULT ram_disk_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count){return STA_NOINIT;};
static DRESULT ram_disk_ioctl(BYTE lun, BYTE cmd, void *buff){return STA_NOINIT;};

static DSTATUS usb_disk_initialize(BYTE lun){return STA_NOINIT;};
static DSTATUS usb_disk_status(BYTE lun){return STA_NOINIT;};
static DRESULT usb_disk_read(BYTE lun, BYTE *buff, DWORD sector, UINT count){return STA_NOINIT;};
static DRESULT usb_disk_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count){return STA_NOINIT;};
static DRESULT usb_disk_ioctl(BYTE lun, BYTE cmd, void *buff){return STA_NOINIT;};


DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		result = ram_disk_status(0);

		// translate the reslut code here

		return stat=result;

	case DEV_MMC :
		result = microsd_disk_status(0);

		// translate the reslut code here

		return stat=result;

	case DEV_USB :
		result = usb_disk_status(0);

		// translate the reslut code here

		return stat=result;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		result = ram_disk_initialize(0);

		// translate the reslut code here

		return stat=result;

	case DEV_MMC :
		result = microsd_disk_initialize(0);

		// translate the reslut code here

		return stat=result;

	case DEV_USB :
		result = usb_disk_initialize(0);

		// translate the reslut code here

		return stat=result;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		result = ram_disk_read(0,buff, sector, count);

		// translate the reslut code here

		return res=result;

	case DEV_MMC :
		// translate the arguments here

		result = microsd_disk_read(0,buff, sector, count);

		// translate the reslut code here

		return res=result;

	case DEV_USB :
		// translate the arguments here

		result = usb_disk_read(0,buff, sector, count);

		// translate the reslut code here

		return res=result;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		result = ram_disk_write(0,buff, sector, count);

		// translate the reslut code here

		return res=result;

	case DEV_MMC :
		// translate the arguments here

#if _USE_WRITE == 1
		result = microsd_disk_write(0,buff, sector, count);

		// translate the reslut code here

#endif
		return res=result;

	case DEV_USB :
		// translate the arguments here

		result = usb_disk_write(0,buff, sector, count);

		// translate the reslut code here

		return res=result;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;

	case DEV_MMC :

		// Process of the command for the MMC/SD card
#if _USE_IOCTL == 1
                res = microsd_disk_ioctl(0, cmd, buff);
#endif
		return res;

	case DEV_USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used 
  * @retval DSTATUS: Operation status
  */
static DSTATUS microsd_disk_initialize(BYTE lun)
{
  Stat = STA_NOINIT;
  
  /* Configure the uSD device */
  if(microsd_init() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
static DSTATUS microsd_disk_status(BYTE lun)
{
  Stat = STA_NOINIT;

  if(microsd_get_card_state() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }
  
  return Stat;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
static DRESULT microsd_disk_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timeout = 100000;

  __disable_irq();
  
  if(microsd_read_blocks((uint32_t*)buff, 
                       (uint32_t) (sector), 
                       count, SD_DATATIMEOUT) == MSD_OK)
  {
    while(microsd_get_card_state()!= MSD_OK)
    {
      if (timeout-- == 0)
      {
        return RES_ERROR;
      }
    }
    res = RES_OK;
  }
  
  __enable_irq();
  return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
static DRESULT microsd_disk_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timeout = 100000;

  __disable_irq();
  
  if(microsd_write_blocks((uint32_t*)buff, 
                        (uint32_t)(sector), 
                        count, SD_DATATIMEOUT) == MSD_OK)
  {
    while(microsd_get_card_state()!= MSD_OK)
    {
      if (timeout-- == 0)
      {
        return RES_ERROR;
      }
    }    
    res = RES_OK;
  }
  
  __enable_irq();
  
  return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
static DRESULT microsd_disk_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  microsd_card_info_t CardInfo;
  
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;
  
  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    microsd_get_card_info(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;
  
  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    microsd_get_card_info(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;
  
  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    microsd_get_card_info(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize;
    break;
  
  default:
    res = RES_PARERR;
  }
  
  return res;
}
#endif