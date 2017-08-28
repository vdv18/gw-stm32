/**
  ******************************************************************************
  * @file    timer_hw_defined.h
  * @author   Dmitry Vakhrushev (vdv.18@mail.ru)
  * @version V1.0
  * @date    12 дек. 2014 г.
  * @brief   
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Copyright (C) 2014 by  Dmitry Vakhrushev (vdv.18@mail.ru) </center></h2>
  ******************************************************************************
  */ 
#ifndef TIMER_HW_DEFINED_H_
#define TIMER_HW_DEFINED_H_

/**
 * Обработчик таймера каждые TIMER_TICK_MS милисекунд
 */
#define TIMER_TICK_MS          			        (1UL)
//125
#define TIMER_MILLISECOND(t)            		( ((t)>=TIMER_TICK_MS)?((t)/TIMER_TICK_MS + ((t)%TIMER_TICK_MS + TIMER_TICK_MS - 1)/TIMER_TICK_MS):TIMER_TICK_MS )
#define TIMER_SECOND(t)                 		( (t)*(1000/TIMER_TICK_MS) )
#define TIMER_MINUTE(t)                 		( (t)*TIMER_SECOND(60) )
#define TIMER_HOURS(t)                  		( (t)*TIMER_MINUTE(60) )

/**
 * Расчетные параметры для милисекундного таймера TIMER1,TIMER2
 * 16 000 000 / 2^7 = 125 000 (1 s) ~ 125 (1 ms)
 */
#ifdef defined(NRF51822) || defined(NRF51422)
#define TIMER_OSCILATOR                                 16000000UL
#define TIMER_PRESCALER                                 7UL
#define TIMER_COUNTER_CALC(OSC,PRESC,MS)                ((((OSC)/(1UL<<(TIMER_PRESCALER)))*MS)/1000UL)
#define TIMER_COUNT                                     TIMER_COUNTER_CALC(TIMER_OSCILATOR,TIMER_PRESCALER,TIMER_TICK_MS)


#define TIMER_HW_INIT()\
  {\
    if((NRF_CLOCK->HFCLKRUN & (1 << 16)) == 0)\
    {\
      NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;\
      NRF_CLOCK->TASKS_HFCLKSTART    = 1;\
      while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {};\
    }\
    if(1)\
    {\
      NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;\
      NRF_TIMER2->PRESCALER = TIMER_PRESCALER;\
      NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;\
      NRF_TIMER2->TASKS_CLEAR = 1;\
      NVIC_ClearPendingIRQ(TIMER2_IRQn);\
      NVIC_SetPriority(TIMER2_IRQn, 4);\
      NVIC_EnableIRQ(TIMER2_IRQn);\
      NRF_TIMER2->CC[0] = TIMER_COUNT;\
      NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;\
      NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);\
      NRF_TIMER2->TASKS_START = 1;\
    }\
  }
//{\
//    if((NRF_CLOCK->HFCLKRUN & (1 << 16)) == 0)\
//    {\
//      NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;\
//      NRF_CLOCK->TASKS_HFCLKSTART    = 1;\
//      while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {};\
//    }\
//    if(1)\
//    {\
//      NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;\
//      NRF_TIMER2->PRESCALER = TIMER_PRESCALER;\
//      NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;\
//      NRF_TIMER2->TASKS_CLEAR = 1;\
//      NVIC_ClearPendingIRQ(TIMER2_IRQn);\
//      NVIC_SetPriority(TIMER2_IRQn, 4);\
//      NVIC_EnableIRQ(TIMER2_IRQn);\
//      NRF_TIMER2->CC[0] = TIMER_COUNT;\
//      NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;\
//      NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);\
//      NRF_TIMER2->TASKS_START = 1;\
//    }\
//}

#define TIMER_HANDLE() void TIMER2_IRQHandler()

#elif defined(STM32L476xx)
#include "stm32l4xx.h"
#include "core_cm4.h"
#include "system_stm32l4xx.h"
#define TIMER_OSCILATOR                                 SystemCoreClock
#define TIMER_PRESCALER                                 7UL
#define TIMER_COUNTER_CALC(OSC,PRESC,MS)                ((((OSC)/(1UL<<(TIMER_PRESCALER)))*MS)/1000UL)
#define TIMER_COUNT                                     TIMER_COUNTER_CALC(TIMER_OSCILATOR,TIMER_PRESCALER,TIMER_TICK_MS)

#define TIMER_HW_INIT()\
  {\
    if (SysTick_Config(SystemCoreClock / 1000 * TIMER_TICK_MS) != 0)  {\
      while(1);\
    }\
  }

#define TIMER_HANDLE() void SysTick_Handler()

#else

#endif


#endif /* TIMER_HW_DEFINED_H_ */
