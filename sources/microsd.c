#include "microsd.h"
SD_HandleTypeDef uSdHandle;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STM32L476G_EVAL_SD_Private_Functions Private Functions
  * @{
  */
static void SD_MspInit(void);
static void SD_MspDeInit(void);
HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup STM32L476G_EVAL_SD_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the SD card device.
  * @param  None
  * @retval SD status
  */
uint8_t microsd_init(void)
{ 
  static uint8_t sd_state_old = MSD_ERROR;
  uint8_t sd_state = MSD_OK;
  if(sd_state_old == MSD_OK)
  {
    return MSD_OK;
  }
  /* uSD device interface configuration */
  uSdHandle.Instance = SDMMC1;
  uSdHandle.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  uSdHandle.Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
  uSdHandle.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  uSdHandle.Init.BusWide             = SDMMC_BUS_WIDE_1B;
  uSdHandle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  uSdHandle.Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;
  
  /* Msp SD initialization */
  SD_MspInit();

  /* HAL SD initialization */
  if(HAL_SD_Init(&uSdHandle) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }
  
  /* Configure SD Bus width */
  if(sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if(HAL_SD_ConfigWideBusOperation(&uSdHandle, SDMMC_BUS_WIDE_4B) != HAL_OK)
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      sd_state = MSD_OK;
    }
  }
  
  return  sd_state;
}

/**
  * @brief  DeInitializes the SD card device.
  * @param  None 
  * @retval SD status
  */
uint8_t microsd_deinit(void)
{ 
  uint8_t sd_state = MSD_OK;
  
  uSdHandle.Instance = SDMMC1;    
  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&uSdHandle) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }
    
  /* Msp SD deinitialization */
  SD_MspDeInit();    

  return  sd_state;
}


/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t microsd_read_blocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state =  HAL_SD_ReadBlocks(&uSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout);

  if  (sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode. 
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t microsd_write_blocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state = HAL_SD_WriteBlocks(&uSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @retval SD status
  */
uint8_t microsd_read_blocks_dma(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;
  
  /* Invalidate the dma tx handle*/
  uSdHandle.hdmatx = NULL;
    
  /* Prepare the dma channel for a read operation */
  sd_state = SD_DMAConfigRx(&uSdHandle);
  
  if(sd_state == HAL_OK)
  {
    /* Read block(s) in DMA transfer mode */
    sd_state = HAL_SD_ReadBlocks_DMA(&uSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  }
  
  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  NumOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t microsd_write_blocks_dma(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;
  
  /* Invalidate the dma rx handle*/
  uSdHandle.hdmarx = NULL;
    
  /* Prepare the dma channel for a read operation */
  sd_state = SD_DMAConfigTx(&uSdHandle);
  
  if(sd_state == HAL_OK)
  { 
    /* Write block(s) in DMA transfer mode */
    sd_state = HAL_SD_WriteBlocks_DMA(&uSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  }
  
  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }
}

/**
  * @brief  Erases the specified memory area of the given SD card. 
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t microsd_erase(uint32_t StartAddr, uint32_t EndAddr)
{
  HAL_StatusTypeDef  sd_state = HAL_OK;

  sd_state = HAL_SD_Erase(&uSdHandle, StartAddr, EndAddr);

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  } 
}

/**
  * @brief  Handles SD card interrupt request.
  * @retval None
  */
void microsd_irq_handler(void)
{
  HAL_SD_IRQHandler(&uSdHandle);
}

/**
  * @brief  Handles SD DMA Tx transfer interrupt request.
  * @retval None
  */
void microsd_txdma_irq_handler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmatx); 
}

/**
  * @brief  Handles SD DMA Rx transfer interrupt request.
  * @retval None
  */
void microsd_rxdma_irq_handler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  */
uint8_t microsd_get_card_state(void)
{
  HAL_SD_CardStateTypedef card_state;
  card_state = HAL_SD_GetCardState(&uSdHandle);
  
  if (card_state == HAL_SD_CARD_TRANSFER)
  {
    return (SD_TRANSFER_OK);
  }
  else if ((card_state == HAL_SD_CARD_SENDING) || 
           (card_state == HAL_SD_CARD_RECEIVING) || 
           (card_state == HAL_SD_CARD_PROGRAMMING))
  {
    return (SD_TRANSFER_BUSY);
  }
  else
  { 
    return(SD_TRANSFER_ERROR);
  }
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None 
  */
void microsd_get_card_info(microsd_card_info_t *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&uSdHandle, CardInfo);
}

/**
  * @}
  */


/** @addtogroup STM32L476G_EVAL_SD_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the SD MSP.
  * @note   The SDMMC clock configuration done within this function assumes that
  *         the PLLSAI1 input clock runs at 8 MHz.
  * @retval None
  */
static void SD_MspInit(void)
{
  GPIO_InitTypeDef gpioinitstruct = {0};
  RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;

  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);

  /* Configure the Eval SDMMC1 clock source. The clock is derived from the PLLSAI1 */
  /* Hypothesis is that PLLSAI1 VCO input is 8Mhz */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  RCC_PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  RCC_PeriphClkInit.PLLSAI1.PLLSAI1Q = 4;
  RCC_PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  RCC_PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit) != HAL_OK)
  {
    while(1) {}
  }
  
  /* Enable SDMMC1 clock */
  __HAL_RCC_SDMMC1_CLK_ENABLE();
  
  /* Enable DMA2 clocks */
  __DMAx_TxRx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Common GPIO configuration */
  gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull      = GPIO_PULLUP;
  gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate = GPIO_AF12_SDMMC1;
  
  /* GPIOC configuration */
  gpioinitstruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
   
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);

  /* GPIOD configuration */
  gpioinitstruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpioinitstruct);

    
  /* NVIC configuration for SDMMC1 interrupts */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  
  /* DMA initialization should be done here but , as there is only one channel for RX and TX it is configured and done directly when required*/
}

/**
  * @brief  De-Initializes the SD MSP.
  * @retval None
  */
static void SD_MspDeInit(void)
{
  DMA_HandleTypeDef hdma;

  /* Disable all interrupts */
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Channel4_IRQn);
  
  /* De-initialize all pins */
  HAL_GPIO_DeInit(GPIOC, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12));
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
  
  /* De-initialize DMA channel */
  hdma.Instance = DMA2_Channel4;
  HAL_DMA_DeInit(&hdma);
  
  /* Deactivate clock of SDMMC */
  __HAL_RCC_SDMMC1_CLK_DISABLE();

}

/**
  * @brief Configure the DMA to receive data from the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd)
{
  static DMA_HandleTypeDef hdma_rx;
  HAL_StatusTypeDef status = HAL_ERROR;
  
  /* Configure DMA Rx parameters */
  hdma_rx.Init.Request             = DMA_REQUEST_7;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

  hdma_rx.Instance = DMA2_Channel4;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, hdma_rx);

  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_rx);
  
  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_rx);

  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_rx);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  
  return (status);
}

/**
  * @brief Configure the DMA to transmit data to the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd)
{
  static DMA_HandleTypeDef hdma_tx;
  HAL_StatusTypeDef status;
  
  /* Configure DMA Tx parameters */
  hdma_tx.Init.Request             = DMA_REQUEST_7;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  
  hdma_tx.Instance = DMA2_Channel4;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, hdma_tx);
  
  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_tx);
  
  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_tx);
  
  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_tx); 
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

  return (status);
}