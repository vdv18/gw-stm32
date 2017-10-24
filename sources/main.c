#include "stm32l4xx.h"
#include "timer.h"
#include "timer_hw_defined.h"

#include <stdlib.h>
#include <string.h>
#include "ff.h"

#ifdef DEBUG
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>
#include "list.h"
static list_head_t message_out;
#endif


#define USE_LPUART1
//#define _TEST_
#ifdef _TEST_
void SysTick_Handler()
{
  
  HAL_IncTick();
  timer_tick();
}
#endif
static timer_t timer_1s = 0;
static timer_t timer_2s = 0;
static timer_t timer_5s = 0;
static timer_t timer_30s = 0;
void send();
static char txbuffer[0x256];
volatile int txlen = 0;
char put_char(char ch);
void timer_cb(timer_t id)
{
  if(id == timer_2s)
  {
    if(txlen)
    {
      char *txp = txbuffer;
      while(*txp != 0)
      {
        put_char(*txp++);
      }
      txlen = 0;
      memset(txbuffer,0,sizeof(txbuffer));
    }
  }
  else
  if(id == timer_1s)
  {
    static uint8_t state = 0;
    switch(state)
    {
      case 0:
        GPIOA->ODR |= GPIO_ODR_OD7;
        state = 1;
        break;
      case 1:
        GPIOA->ODR |= GPIO_ODR_OD7;
        state = 2;
        break;
      case 2:
        GPIOA->ODR &=~GPIO_ODR_OD7;
        state = 3;
        timer_stop(timer_1s);
        timer_start(timer_5s);
        break;
    default:
      break;
    }
  }
  else
  if(id == timer_5s)
  {
    send();
  }
}

#define WAIT_FLAG(EXP) {\
  volatile unsigned int delay = 1000000;\
  while( (EXP) && (delay > 1)){\
    dealy--;\
  }\
}
volatile int temp = 0;
volatile int sleep = 0;
#define RTC_WKUP_EXTI (1<<20)

char put_rs_char(char ch);
void RTC_WKUP_IRQHandler()
{
  static char ch = 0;
  if (RTC->ISR & RTC_ISR_WUTF) {
    //sleep = ~sleep;
    //timer_tick_diff(1000);
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
    PWR->CR1  |= PWR_CR1_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
    RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
    PWR->CR1  &= ~PWR_CR1_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
    EXTI->PR1 |=  RTC_WKUP_EXTI;
    put_rs_char('0'+ch++);
    if(ch>9)ch=0;
  }
}
static uint8_t modem_work = 0;
static uint8_t rx_buffer[0x1000];
static uint8_t *prx_buffer;
static uint8_t buffer_len = 0;
#ifdef USE_LPUART1
void LPUART1_IRQHandler(void)
{
  if(LPUART1->ISR & USART_ISR_RXNE)
  {
    *prx_buffer++ = LPUART1->RDR;
    buffer_len++;
  }
  if(LPUART1->ISR & USART_ISR_IDLE)
  {
#ifdef DEBUG
    list_t *item;
#endif
    prx_buffer = rx_buffer;
    
    if(strstr(rx_buffer,"\r\nOK\r\n") != NULL)
    {
      modem_work = 1;
      //timer_stop(timer_5s);
    }
#ifdef DEBUG
    item = malloc(sizeof(item));
    if(item)
    {
      item->next = 0;
      item->data = malloc(buffer_len);
      if(item->data)
      {
        list_push_first(&message_out, item);
        sprintf((char*)item->data, "%s",rx_buffer);
      }
      else
      {
        free(item);
      }
    }
#endif
    memset(rx_buffer,0,sizeof(rx_buffer));
    buffer_len = 0;
    LPUART1->ICR |= 1<<4;
  }
  LPUART1->ICR |= USART_ICR_PECF;
  LPUART1->ICR |= USART_ICR_FECF;
  LPUART1->ICR |= USART_ICR_ORECF;
}
#else
void USART3_IRQHandler(void)
{
  if(USART3->ISR & USART_ISR_RXNE)
  {
    *prx_buffer = USART3->RDR;
    buffer_len++;
  }
  if(USART3->ISR & USART_ISR_IDLE)
  {
    prx_buffer = rx_buffer;
    memset(rx_buffer,0,sizeof(rx_buffer));
    buffer_len = 0;
    USART3->ICR |= 1<<4;
  }
  //USART3->ICR |= 1<<1;
}
#endif

static uint8_t uart4_test[10] = 0;
void UART4_IRQHandler(void)
{
  static uint8_t *p = uart4_test;
  if(UART4->ISR & USART_ISR_RXNE)
  {
    if((p >= &uart4_test[10]) || (p < &uart4_test[0])) p = &uart4_test[0];
    *p++ = UART4->RDR;
  }
  if(UART4->ISR & USART_ISR_IDLE)
  {
    UART4->ICR |= 1<<4;
  }
}
char put_rs_char(char ch){
  while ((UART4->ISR & USART_ISR_TXE) == 0){}
  UART4->TDR = (ch);
  return (ch);
}
char put_char(char ch){

#ifdef USE_LPUART1
	//Wait for buffer to be empty
  while ((LPUART1->ISR & USART_ISR_TXE) == 0){}
	
	//Send character
  LPUART1->TDR = (ch);
#else
	//Wait for buffer to be empty
  while ((USART3->ISR & USART_ISR_TXE) == 0){}
	
	//Send character
  USART3->TDR = (ch);
  
#endif

  return (ch);
}
void send()
{
  static int first = 1;
  LPUART1->CR1 |= USART_CR1_UE;
  USART3->CR1 |= USART_CR1_UE;
  switch(first++)
  {
  case 0:
    put_char('A');
    put_char('T');
    put_char('+');
    put_char('C');
    put_char('L');
    put_char('T');
    put_char('S');
    put_char('=');
    put_char('1');
    put_char('\r');
    put_char('\n');
    break;
  case 1:
    if(!modem_work)
      first = 0;
    put_char('A');
    put_char('T');
    put_char('+');
    put_char('C');
    put_char('C');
    put_char('L');
    put_char('K');
    put_char('?');
    put_char('\r');
    put_char('\n');
    break;
  case 2:
    put_char('A');
    put_char('T');
    put_char('+');
    put_char('S');
    put_char('M');
    put_char('T');
    put_char('P');
    put_char('F');
    put_char('I');
    put_char('L');
    put_char('E');
    put_char('=');
    put_char('?');
    put_char('\r');
    put_char('\n');

    break;
  case 3:
    first = 1;
    put_char('A');
    put_char('T');
    put_char('+');
    put_char('S');
    put_char('M');
    put_char('T');
    put_char('P');
    put_char('F');
    put_char('T');
    put_char('=');
    put_char('?');
    put_char('\r');
    put_char('\n');
    first = 4;
    break;
  case 4:
    break;
  }
}

void uart_init()
{  
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  prx_buffer = rx_buffer;
  RCC->CCIPR = 0x00;//(0x03<<10);
#ifdef _TEST_
  RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_0;
#else
  RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_0;
#endif
  //RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1;
  GPIOB->MODER &=~( GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
  GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  
#ifdef USE_LPUART1
  GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOB->AFR[1] |= (GPIO_AFRL_AFSEL2_3 | GPIO_AFRL_AFSEL3_3);
  
  LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
#ifdef _TEST_
  LPUART1->BRR = 256*4000000/57600;////256*4000000UL/9600UL;//4000000/115200;//
#else
  LPUART1->BRR = 0x56CE3>>1;//256*80000000UL/57600;////256*4000000UL/9600UL;//4000000/115200;//
#endif
  //LPUART1->CR2 = USART_CR2_SWAP;
  //LPUART1->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
#else
  GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOB->AFR[1] |= (GPIO_AFRL_AFSEL2 & ~(GPIO_AFRL_AFSEL2_3)) | (GPIO_AFRL_AFSEL3 & ~(GPIO_AFRL_AFSEL3_3));
  
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
#ifdef _TEST_
  USART3->BRR = 0x1A1;
#else
  USART3->BRR = 8333;
#endif
  
#endif
  
}
void sleep_deep()
{
  //
  RCC->APB1RSTR2 |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN  | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOGEN;
  {volatile uint32_t tr=9999999;while(tr){tr--;};}
  GPIOA->MODER &=~( GPIO_MODER_MODE2 );
  GPIOA->PUPDR &=~( GPIO_PUPDR_PUPD2 );
  GPIOA->PUPDR |= ( GPIO_PUPDR_PUPD2_1 );
  
  EXTI->PR1    =  (1<<2); // Clear IT pending bit
  EXTI->IMR1  |=  (1<<2); // Enable interrupt request from EXTI line
  EXTI->EMR1  &= ~(1<<2); // Disable event on EXTI line
  EXTI->RTSR1 |=  (1<<2); // Trigger rising edge enabled
  EXTI->FTSR1 &= ~(1<<2); // Trigger falling edge disabled
  
  PWR->CR3 |= PWR_CR3_EWUP4;// | PWR_CR3_EWUP | PWR_CR3_APC | PWR_CR3_EIWF;
  
  /* Set Shutdown mode */
  PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN;
  
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Request Wait For Interrupt */
  __WFE();
  __WFE();
}
void uart_rs_init()
{  
  RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
  //GPIOA->ODR |= 0<<8;
  GPIOA->MODER &=~( GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE6 | GPIO_MODER_MODE8);
  GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE6_0;
  GPIOB->MODER &=~( GPIO_MODER_MODE14 );
  
  GPIOB->MODER &=~( GPIO_MODER_MODE14 );
  GPIOB->MODER |= GPIO_MODER_MODE14_0;
  GPIOB->ODR |= 1<<14;
  
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL0_3) | (GPIO_AFRL_AFSEL1_3);
  
  UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
#ifdef _TEST_
  UART4->BRR = 0x1A1;  
#else
  UART4->BRR = 8333; 
#endif
  UART4->CR1 |= USART_CR1_UE;
}
#ifdef _TEST_

void main()
{
  volatile int temp = 9999;
  //while(1);
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN  | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOGEN;
  GPIOA->MODER &=~( GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE6 | GPIO_MODER_MODE8);
  GPIOA->MODER |= GPIO_MODER_MODE8_0;
  while(temp>0){temp--;};
  GPIOA->ODR &=~(1<<8);
  timer_init(); 
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  PWR->CR1 |= PWR_CR1_DBP;
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  RCC->BDCR = RCC_BDCR_BDRST;
  RCC->BDCR &=~RCC_BDCR_BDRST;
  RCC->BDCR |= RCC_BDCR_LSEDRV_1 | RCC_BDCR_LSEDRV_0;
  RCC->BDCR |= RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSECSSD;
  RCC->BDCR |= RCC_BDCR_LSEON;
  temp = RCC->BDCR;
  //WAIT_FLAG( ! RCC->BDCR & RCC_BDCR_LSERDY);
  while(!( temp = RCC->BDCR & RCC_BDCR_LSERDY) ){};
  RCC->BDCR |= RCC_BDCR_RTCEN;
  
  EXTI->PR1    =  RTC_WKUP_EXTI; // Clear IT pending bit
  EXTI->IMR1  |=  RTC_WKUP_EXTI; // Enable interrupt request from EXTI line
  EXTI->EMR1  &= ~RTC_WKUP_EXTI; // Disable event on EXTI line
  EXTI->RTSR1 |=  RTC_WKUP_EXTI; // Trigger rising edge enabled
  EXTI->FTSR1 &= ~RTC_WKUP_EXTI; // Trigger falling edge disabled
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &=~RTC_CR_WUTE;
  
  while(! RTC->ISR & RTC_ISR_WUTWF){};
  
  RTC->WUTR = 0x0000;
  RTC->CR &= ~RTC_CR_WUCKSEL;
  RTC->CR |=  RTC_CR_WUCKSEL_2;
  RTC->CR |= RTC_CR_WUTIE;
  RTC->CR |= RTC_CR_WUTE;
  RTC->WPR = 0xFF;
    
  RTC->ISR &=~RTC_ISR_RSF;
  RTC->ISR &=~RTC_ISR_WUTF;
  RTC->ISR = 0;
  RTC->CR &=~RTC_CR_WUTE;
  RTC->CR = 0;
  NVIC_EnableIRQ(LPUART1_IRQn);
  NVIC_EnableIRQ(UART4_IRQn);
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
  RTC->CR |= RTC_CR_WUTIE;
  RTC->CR |= RTC_CR_WUTE;
  
  GPIOC->MODER  &= ~(GPIO_MODER_MODE5);
  GPIOC->MODER  |= (GPIO_MODER_MODE5_0);
  
  GPIOA->MODER  &= ~(GPIO_MODER_MODE7);
  GPIOA->MODER  |= (GPIO_MODER_MODE7_0);
  
  
  GPIOC->ODR |= GPIO_ODR_OD5;
  
  uart_init();
  uart_rs_init();
  SysTick->LOAD = SystemCoreClock / 1000 - 1; // 1 ms
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  timer_5s = timer_create(TIMER_REPEAT, TIMER_SECOND(2), timer_cb);
  timer_1s = timer_create(TIMER_REPEAT_START, TIMER_SECOND(1), timer_cb);
  timer_2s = timer_create(TIMER_REPEAT_START, TIMER_SECOND(2), timer_cb);
  
#ifdef DEBUG
  printf("Init CPU\r\n");
#endif
  while(1){
#ifdef DEBUG
     list_t *item;
#endif
#ifdef DEBUG  
    item = list_pop_last(&message_out);
    if(item)
    { 
      if(item->data)printf((char*)item->data);
      if(item->data)free(item->data);
      free(item);
    }
#endif
    timer_handle();
    if(sleep)
      __WFE();
  }
}
#else

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage.h"
#include "stm32l4xx_hal.h"
static void SystemClock_Config(void);
static void Error_Handler(void);
USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;


void SysTick_Handler()
{
  HAL_IncTick();
  timer_tick();
  HAL_SYSTICK_IRQHandler();
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
static uint8_t rtext[100];    
static void timer_cb1(timer_t id)
{
  if(id == timer_5s)
  {
    if(f_mount(&SDFatFs, "1:/", 0) == FR_OK)
    {
      if(f_open(&MyFile, "1:/STM32.TXT", FA_READ) == FR_OK)
      {
        FRESULT res;
        uint32_t bytesread; 
        memset(rtext,0,sizeof(rtext));
        res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
        if((bytesread != 0) && (res == FR_OK))
        {
          printf("%s\r\n",rtext);
        }
        f_close(&MyFile);
      }
    }
    
  }
}

static SPI_HandleTypeDef spi = { .Instance = SPI1 };
int spi_init(void)
{
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __SPI1_CLK_ENABLE();
  spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi.Init.Direction = SPI_DIRECTION_2LINES;
  spi.Init.CLKPhase = SPI_CR1_CPHA;
  spi.Init.CLKPolarity = SPI_CR1_CPOL;
  spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  spi.Init.DataSize = SPI_DATASIZE_8BIT;
  spi.Init.FirstBit = SPI_FIRSTBIT_LSB;
  spi.Init.NSS = SPI_NSS_SOFT;
  spi.Init.TIMode = SPI_TIMODE_DISABLED;
  spi.Init.Mode = SPI_MODE_MASTER; 
  if (HAL_SPI_Init(&spi) != HAL_OK)
  {
    asm("bkpt 255");
  }
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIOA->ODR &=~(1<<4);
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  
  
//  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_4 | GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 

  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
timer_t timer_spi, timer_xs;
static uint8_t temp_out[0x80] = {0x00,0x00,0x00,0x44};
static uint8_t temp_in[0x80] = {0x00,0x00,0x00,0x00};
static int sleep_start = 0;
static void timer_cb2(timer_t id)
{
  if(id == timer_30s)
  {
    sleep = 1;
  }
  if(id == timer_xs)
  {
    //timer_start(timer_spi);
  }
  if(id == timer_spi)
  {
    static int fsm = 0;
    switch(fsm++){
      case 0:
        if(temp_in[1] == 3)
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        if(RCC->BDCR & RCC_BDCR_LSERDY && RCC->CR & RCC_CR_HSERDY)
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        if(modem_work)
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        if(temp_in[1] == 3)
        {
          if(RCC->BDCR & RCC_BDCR_LSERDY && RCC->CR & RCC_CR_HSERDY)
          {
            if(modem_work)
            {
              if(!sleep_start)
              {
                sleep_start = 1;
                timer_start(timer_30s);
              }
            }
          }
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        break;
      case 1:
        if(sleep_start)
          temp_out[1] = 1;
        HAL_SPI_TransmitReceive(&spi, (uint8_t *)&temp_out, (uint8_t*)&temp_in, 4, 100);
        break;
      case 2:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        break;
      case 3:
        static int temp = 0;
        if(sleep)
        {
          temp++;
          if(temp > 100)
          {
            temp =0;
            //sleep_deep();
          }
        }
        break;
      default:
        if(temp_in[0] == 0x01)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        } else
        if(temp_in[0] == 0x02)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        } else
        if(temp_in[0] == 0x03)
        {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        } else
        if(temp_in[0] == 0x04)
        {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
        //timer_start(timer_spi);
        //timer_stop(timer_spi);
        fsm = 0;
        break;
    };
  }
}
int test_init()
{
  
  
  //while(1);
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN  | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOGEN;
  GPIOB->MODER &=~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
  GPIOB->MODER |= GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0;
  GPIOA->MODER &=~( GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE6 | GPIO_MODER_MODE8 | GPIO_MODER_MODE15);
  GPIOA->MODER |= GPIO_MODER_MODE8_0;
  GPIOA->MODER |= GPIO_MODER_MODE15_0;
  GPIOA->ODR &=~(1<<15);
  {volatile int temp = 9999999;while(temp>0){temp--;};}
  GPIOA->ODR |= (1<<15);
  GPIOA->MODER &=~(GPIO_MODER_MODE15);
  GPIOA->ODR &=~(1<<8);
  //timer_init(); 
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  PWR->CR1 |= PWR_CR1_DBP;
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  RCC->BDCR = RCC_BDCR_BDRST;
  RCC->BDCR &=~RCC_BDCR_BDRST;
  RCC->BDCR |= RCC_BDCR_LSEDRV_1 | RCC_BDCR_LSEDRV_0;
  RCC->BDCR |= RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSECSSD;
  RCC->BDCR |= RCC_BDCR_LSEON;
  temp = RCC->BDCR;
  //WAIT_FLAG( ! RCC->BDCR & RCC_BDCR_LSERDY);
  while(!( temp = RCC->BDCR & RCC_BDCR_LSERDY) ){};
  RCC->BDCR |= RCC_BDCR_RTCEN;
  
  EXTI->PR1    =  RTC_WKUP_EXTI; // Clear IT pending bit
  EXTI->IMR1  |=  RTC_WKUP_EXTI; // Enable interrupt request from EXTI line
  EXTI->EMR1  &= ~RTC_WKUP_EXTI; // Disable event on EXTI line
  EXTI->RTSR1 |=  RTC_WKUP_EXTI; // Trigger rising edge enabled
  EXTI->FTSR1 &= ~RTC_WKUP_EXTI; // Trigger falling edge disabled
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &=~RTC_CR_WUTE;
  
  while(! RTC->ISR & RTC_ISR_WUTWF){};
  
  RTC->WUTR = 0x0000;
  RTC->CR &= ~RTC_CR_WUCKSEL;
  RTC->CR |=  RTC_CR_WUCKSEL_2;
  //RTC->CR |= RTC_CR_WUTIE;
  //RTC->CR |= RTC_CR_WUTE;
  RTC->WPR = 0xFF;
    
  RTC->ISR &=~RTC_ISR_RSF;
  RTC->ISR &=~RTC_ISR_WUTF;
  RTC->ISR = 0;
  RTC->CR &=~RTC_CR_WUTE;
  RTC->CR = 0;
  NVIC_EnableIRQ(LPUART1_IRQn);
  NVIC_EnableIRQ(UART4_IRQn);
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
  //RTC->CR |= RTC_CR_WUTIE;
  //RTC->CR |= RTC_CR_WUTE;
  
  GPIOC->MODER  &= ~(GPIO_MODER_MODE5);
  GPIOC->MODER  |= (GPIO_MODER_MODE5_0);
  
  GPIOA->MODER  &= ~(GPIO_MODER_MODE7);
  GPIOA->MODER  |= (GPIO_MODER_MODE7_0);
  
  
  GPIOC->ODR |= GPIO_ODR_OD5;
  
  uart_init();
  uart_rs_init();
  //SysTick->LOAD = 4000000 / 1000 - 1; // 1 ms
  //SysTick->VAL = 0;
  //SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  timer_5s = timer_create(TIMER_REPEAT, TIMER_SECOND(2), timer_cb);
  timer_1s = timer_create(TIMER_REPEAT_START, TIMER_SECOND(1), timer_cb);
  timer_2s = timer_create(TIMER_REPEAT_START, TIMER_SECOND(2), timer_cb);
#undef DEBUG
#ifdef DEBUG
  printf("Init CPU\r\n");
#endif
}
int test()
{
  while(1){
#ifdef DEBUG
     list_t *item;
#endif
#ifdef DEBUG  
    item = list_pop_last(&message_out);
    if(item)
    { 
      if(item->data)printf((char*)item->data);
      if(item->data)free(item->data);
      free(item);
    }
#endif
    timer_handle();
    if(sleep)
      sleep_deep();
  }
}
int main()
{
  SystemClock_Config();
  RCC->CR |= RCC_CR_HSEON;
  RCC->BDCR |= RCC_BDCR_LSEON;
  timer_init();
  test_init();
  HAL_Init();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();
  
  /* Init MSC Application */
  USBD_Init(&USBD_Device, &MSC_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_MSC_CLASS);
  
  /* Add Storage callbacks for MSC Class */
  USBD_MSC_RegisterStorage(&USBD_Device, &USBD_DISK_fops);
  
    /* Start Device Process */
  USBD_Start(&USBD_Device);
  timer_30s = timer_create(TIMER_ONE_SHOT, TIMER_SECOND(15), timer_cb2);
  timer_xs = timer_create(TIMER_REPEAT_START, TIMER_SECOND(1), timer_cb2);
  timer_spi = timer_create(TIMER_REPEAT, TIMER_MILLISECOND(10), timer_cb2);
  timer_start(timer_spi);
  spi_init();
  
  test();
  
  
  
  while(1)
  {
    timer_handle();
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *
  *         If define USB_USE_LSE_MSI_CLOCK enabled:
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            MSI Frequency(Hz)              = 4800000
  *            LSE Frequency(Hz)              = 32768
  *            PLL_M                          = 6
  *            PLL_N                          = 40
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 4
  *            Flash Latency(WS)              = 4
  * 
  *         If define USB_USE_HSE_CLOCK enabled:
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 1
  *            PLL_N                          = 20
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * 
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

#define USB_USE_LSE_MSI_CLOCK
#if defined (USB_USE_LSE_MSI_CLOCK)
 
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 8;
  RCC_OscInitStruct.PLL.PLLR            = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();

  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  
#elif defined (USB_USE_HSE_CLOCK)
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;  
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1Q = 4; 
  PeriphClkInitStruct.PLLSAI1.PLLSAI1P = 1;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut= RCC_PLLSAI1_48M2CLK;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  
#endif /* USB_USE_LSE_MSI_CLOCK */
  SystemCoreClockUpdate();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while (1)
  {
  }
}
#endif