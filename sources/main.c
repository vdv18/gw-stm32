#include "stm32l4xx.h"
#include "timer.h"
#include "timer_hw_defined.h"

#include <stdlib.h>
#include <string.h>

#ifdef DEBUG
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>
#include "list.h"
static list_head_t message_out;
#endif


#define USE_LPUART1

void SysTick_Handler()
{
  timer_tick();
}
static timer_t timer_1s = 0;
static timer_t timer_2s = 0;
static timer_t timer_5s = 0;
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
void RTC_WKUP_IRQHandler()
{
  if (RTC->ISR & RTC_ISR_WUTF) {
    //sleep = ~sleep;
    timer_tick_diff(1000);
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
    PWR->CR1  |= PWR_CR1_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
    RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
    PWR->CR1  &= ~PWR_CR1_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
    EXTI->PR1 |=  RTC_WKUP_EXTI;
  }
}
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
      timer_stop(timer_5s);
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
  LPUART1->CR1 |= USART_CR1_UE;
  USART3->CR1 |= USART_CR1_UE;
  put_char('A');
  put_char('T');
  put_char('\r');
  put_char('\n');
}

void uart_init()
{  
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  prx_buffer = rx_buffer;
  RCC->CCIPR = 0x00;//(0x03<<10);
  RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_0;
  //RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1;
  GPIOB->MODER &=~( GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
  GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  
#ifdef USE_LPUART1
  GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOB->AFR[1] |= (GPIO_AFRL_AFSEL2_3 | GPIO_AFRL_AFSEL3_3);
  
  LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
  LPUART1->BRR = 256*4000000/57600;////256*4000000UL/9600UL;//4000000/115200;//
  //LPUART1->CR2 = USART_CR2_SWAP;
  //LPUART1->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
#else
  GPIOB->AFR[1] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOB->AFR[1] |= (GPIO_AFRL_AFSEL2 & ~(GPIO_AFRL_AFSEL2_3)) | (GPIO_AFRL_AFSEL3 & ~(GPIO_AFRL_AFSEL3_3));
  
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
  USART3->BRR = 0x1A1;
  
#endif
  
}

void main()
{
  //while(1);
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN  | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOGEN;
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
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
  RTC->CR |= RTC_CR_WUTIE;
  RTC->CR |= RTC_CR_WUTE;
  
  GPIOC->MODER  &= ~(GPIO_MODER_MODE5);
  GPIOC->MODER  |= (GPIO_MODER_MODE5_0);
  
  GPIOA->MODER  &= ~(GPIO_MODER_MODE7);
  GPIOA->MODER  |= (GPIO_MODER_MODE7_0);
  
  
  GPIOC->ODR |= GPIO_ODR_OD5;
  
  uart_init();
  //SysTick->LOAD = 4000000 / 1000 - 1; // 1 ms
  //SysTick->VAL = 0;
  //SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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
