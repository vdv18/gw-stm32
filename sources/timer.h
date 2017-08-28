/**
  ******************************************************************************
  * @file    timer.h
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
#ifndef TIMER_H_
#define TIMER_H_

typedef enum{
	TIMER_OK = 1,
	TIMER_ERR = 0,
} timer_state_t;

typedef enum{
	TIMER_REPEAT,
	TIMER_REPEAT_START,
	TIMER_ONE_SHOT,
	TIMER_ONE_SHOT_START,
	TIMER_ONE_SHOT_DELETE,
	TIMER_ONE_SHOT_DELETE_START,
} timer_mode_t;

typedef void* timer_t;
typedef unsigned long timer_time_t;
typedef void (*timer_callback_t)(timer_t);


timer_state_t timer_init( void );

/**
 * @brief Timer create function
 *
 * @param[in] timer_mode_t 		-	mode of creating timer.
 * @param[in] timer_time_t		-	timer of callback tick.
 * @param[in] timer_callback_t 	-	timer callback.
 *
 * @retval timer_t 		-	pointer to created timer.
 *
 */
timer_t timer_create(timer_mode_t, timer_time_t, timer_callback_t);

/**
 * @brief Timer delete function
 *
 * @param[in] timer_t - pointer to created timer
 *
 * @retval none
 *
 */
timer_state_t timer_delete(timer_t*);

/**
 * @brief Set callback to created timer
 *
 * @param[in] timer_callback_t		-	callback  function
 *
 * @retval TIMER_OK			-	timer set callback
 * @retval TIMER_ERR		-	timer is not created
 *
 */
timer_state_t timer_callback_set(timer_t,timer_callback_t);

/**
 *	@brief Get callback from created timer
 *
 *	@retval timer_callback_t 	-	pointer to callback function
 *
 */
timer_callback_t timer_callback_get(timer_t);

/**
 * @brief Set update time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 * @param[in] timer_time_t	-	update time
 *
 * @retval	none
 *
 */
timer_state_t timer_set_time(timer_t, timer_time_t);

/**
 * @brief Get update time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_get_time(timer_t);


/**
 * @brief Get elapsed time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_elapsed_time(timer_t);

/**
 * @brief Get remaining time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_remaining_time(timer_t);


/**
 * @brief Get timer ready
 *
 * @param[in]	timer_t	-	pointer to a time
 *
 * @retval	TIMER_OK	-	timer is ready
 * @retval 	TIMER_ERR	-	timer is not ready
 *
 */
timer_state_t timer_ready(timer_t);

/**
 * @brief	Get timer enable
 *
 * @param[in]	timer_t	-	pointer to a timer
 *
 * @retval	TIMER_OK	-	timer enable
 * @retval	TIMER_ERR	-	timer disable
 *
 */
timer_state_t timer_enable(timer_t);

/**
 * @brief	Timer start
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_start(timer_t);

/**
 * @brief	Timer stop
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_reset(timer_t);

/**
 * @brief	Timer reset time to zero
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_stop(timer_t);


/**
 * @brief	Timer pause timer
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_pause(timer_t);


/**
 * @brief	Timer continuation timer
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_continuation(timer_t);


/**
 * @brief	Get global timer timestamp
 *
 * @retval	timer_time_t	-	timestamp
 */
timer_time_t timer_timestamp(void);


/**
 *	@brief	Timer tick
 *
 *	@retval none
 *
 */
void timer_tick(void);



/**
 *	@brief	Timer tick
 *
 *      @param	unsigned long	-	elapsed time in ticks
 *
 *	@retval none
 *
 */
void timer_tick_diff(unsigned long elapsed);

/**
 *	@brief	Timer handle
 *
 *	@retval none
 *
 */
void timer_handle(void);


#endif /* TIMER_H_ */
