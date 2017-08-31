#ifndef __MICROSD_H__
#define __MICROSD_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L476G_EVAL
  * @{
  */

/** @addtogroup STM32L476G_EVAL_SD
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_SD_Exported_Types Exported Types
  * @{
  */

/** 
  * @brief SD Card information structure 
  */
#define MICRO_SD_CardInfo HAL_SD_CardInfoTypeDef
/**
  * @}
  */
   
/** 
  * @brief  SD status structure definition  
  */     
#define   MSD_OK                        ((uint8_t)0x00)
#define   MSD_ERROR                     ((uint8_t)0x01)
#define   MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

/** 
  * @brief  SD transfer state definition  
  */     
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)
#define   SD_TRANSFER_ERROR             ((uint8_t)0x02)

/* Exported constants --------------------------------------------------------*/  

/** @defgroup STM32L476G_EVAL_SD_Exported_Constants  Exported Constants
  * @{
  */ 
#define SD_DETECT_PIN                    GPIO_PIN_8
#define SD_DETECT_GPIO_PORT              GPIOA
#define __SD_DETECT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_DETECT_IRQn                   EXTI9_5_IRQn
#define SD_Detect_IRQHandler             EXTI9_5_IRQHandler
   
#define SD_DATATIMEOUT           ((uint32_t)100000000)
    
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
   
/* DMA definitions for SD DMA transfer */
#define __DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define SD_DMAx_Tx_STREAM                 DMA2_Channel4  
#define SD_DMAx_Rx_STREAM                 DMA2_Channel4  
#define SD_DMAx_Tx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Tx_IRQHandler             DMA2_Channel4_IRQHandler
#define SD_DMAx_Rx_IRQHandler             DMA2_Channel4_IRQHandler

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM32L476G_EVAL_SD_Exported_Functions Exported Functions
  * @{
  */
uint8_t MICRO_SD_Init(void);
uint8_t MICRO_SD_DeInit(void);
uint8_t MICRO_SD_ITConfig(void);
uint8_t MICRO_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t MICRO_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t MICRO_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t MICRO_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t MICRO_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
void    MICRO_SD_IRQHandler(void);
void    MICRO_SD_DMA_Tx_IRQHandler(void);
void    MICRO_SD_DMA_Rx_IRQHandler(void);
uint8_t MICRO_SD_GetCardState(void);
void    MICRO_SD_GetCardInfo(MICRO_SD_CardInfo *CardInfo);
uint8_t MICRO_SD_IsDetected(void);


#endif//__MICROSD_H__