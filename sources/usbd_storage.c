/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_storage.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright(c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage.h"
#include "microsd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FOPS_LUN_NBR                  1  
#define FOPS_BLKNBR                  0x10000  
#define FOPS_BLKSIZ                  0x200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* USB Mass storage Standard Inquiry Data */
int8_t fops_inquiry_data[] = { /* 36 */
  /* LUN 0 */
  0x00,		
  0x80,		
  0x02,		
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,	
  0x00,
  'M', 'G', 'T', ' ', ' ', ' ', ' ', ' ', /* Manufacturer: 8 bytes  */
  'B', 'L', 'E', ' ', 'G', 'a', 't', 'e', /* Product     : 16 Bytes */
  'w', 'a', 'y', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0','1',                      /* Version     : 4 Bytes  */
}; 

/* Private function prototypes -----------------------------------------------*/
int8_t fops_init(uint8_t lun);
int8_t fops_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
int8_t fops_is_ready(uint8_t lun);
int8_t fops_is_write_protected(uint8_t lun);
int8_t fops_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t fops_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t fops_get_max_lun(void);

USBD_StorageTypeDef USBD_DISK_fops = {
  fops_init,
  fops_get_capacity,
  fops_is_ready,
  fops_is_write_protected,
  fops_read,
  fops_write,
  fops_get_max_lun,
  fops_inquiry_data, 
};
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initailizes the storage unit (medium)       
  * @param  lun: Logical unit number
  * @retval Status (0 : Ok / -1 : Error)
  */
int8_t fops_init(uint8_t lun)
{
  int8_t ret = -1;  

  if(microsd_init() == MSD_OK)
  {
    ret = 0;
  }
  return ret;
}

/**
  * @brief  Returns the medium capacity.      
  * @param  lun: Logical unit number
  * @param  block_num: Number of total block number
  * @param  block_size: Block size
  * @retval Status (0: Ok / -1: Error)
  */
int8_t fops_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  micro_sd_card_info info;
  int8_t ret = -1;  
  
  microsd_get_card_info(&info);
  
  *block_num =  info.LogBlockNbr  - 1;
  *block_size = info.LogBlockSize;
  ret = 0;
  return ret;
}

/**
  * @brief  Checks whether the medium is ready.  
  * @param  lun: Logical unit number
  * @retval Status (0: Ok / -1: Error)
  */
int8_t fops_is_ready(uint8_t lun)
{
  int8_t ret = -1;
  if(microsd_get_card_state() == SD_TRANSFER_OK)
  {
    ret = 0;
  }
  return ret;
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number
  * @retval Status (0: write enabled / -1: otherwise)
  */
int8_t fops_is_write_protected(uint8_t lun)
{
  return -1;
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0: Ok / -1: Error)
  */
int8_t fops_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  int8_t ret = -1;  
  uint32_t timeout = 100000;
  
  microsd_read_blocks((uint32_t *)buf, blk_addr, blk_len, SD_DATATIMEOUT);
  while(microsd_get_card_state() != SD_TRANSFER_OK)
  {
    if (timeout-- == 0)
    {
      return ret;
    }
  }
  ret = 0;

  return ret;
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0 : Ok / -1 : Error)
  */
int8_t fops_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  int8_t ret = -1;  
  uint32_t timeout = 100000;  
  
  microsd_write_blocks((uint32_t *)buf, blk_addr, blk_len, SD_DATATIMEOUT);
  while(microsd_get_card_state() != SD_TRANSFER_OK)
  {
    if (timeout-- == 0)
    {
      return ret;
    }
  }
  ret = 0;

  return ret;

}


/**
  * @brief  Returns the Max Supported LUNs.   
  * @param  None
  * @retval Lun(s) number
  */
int8_t fops_get_max_lun(void)
{
  return(FOPS_LUN_NBR - 1);
}
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
