/**
  ******************************************************************************
  * @file    timer.c
  * @author   Dmitry Vakhrushev (vdv.18@mail.ru)
  * @version V1.0
  * @date    12 дек. 2014 г.
  * @brief   
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, TRANSMASH-TOMSK SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Copyright (C) 2014 by  Dmitry Vakhrushev (vdv.18@mail.ru) </center></h2>
  ******************************************************************************
  */ 
#include <stdint.h>
#include "timer.h"
#include "timer_hw_defined.h"
#include "list.h"

#ifndef TIMERS_MAX_CNT
  #define TIMERS_MAX_CNT	50
#endif

typedef struct timer_struct_s {
  timer_callback_t      callback;
  timer_time_t          exptime;
  timer_time_t          timestamp;
  timer_time_t          update_time;
  timer_mode_t          mode;
  union {
    uint32_t   state;
    struct {
      uint32_t allocated:1;
      uint32_t running:1;
      uint32_t pause:1;
      uint32_t ready:1;
      uint32_t callback_immediately:1;
      uint32_t expoverflow:1;
      uint32_t enabled:1;
      uint32_t stop:1;
      uint32_t del:1;
    };
  };
} timer_struct_t;

static timer_struct_t   timers[TIMERS_MAX_CNT];
static timer_time_t     timers_global_tick = 0x00;//0xFFFFFFFF - 10000;
static list_t           timers_all[TIMERS_MAX_CNT];
static list_head_t      timers_free;
static list_head_t      timers_handled;
static list_head_t      timers_not_handled;

static char             timers_initialized = 0;


#define TIMER_CREATE(TIMER,MODE,TIME,RUNNING,CALLBACK)\
	{\
                TIMER->state = 0;\
		TIMER->allocated = 1;\
		TIMER->running = RUNNING;\
		TIMER->mode = MODE;\
		TIMER->update_time = TIME;\
		TIMER->callback = CALLBACK;\
	};
#define TIMER_START(TIMER)\
        {\
                TIMER->timestamp = timer_timestamp();\
                TIMER->exptime = TIMER->timestamp + TIMER->update_time;\
                if(TIMER->exptime < TIMER->timestamp)\
                  {TIMER->expoverflow=1;}else{TIMER->expoverflow=0;}\
        };
#define TIMER_RESET(TIMER)\
        {\
                TIMER->timestamp = timer_timestamp();\
                TIMER->exptime = TIMER->timestamp + TIMER->update_time;\
                if(TIMER->exptime < TIMER->timestamp)\
                  {TIMER->expoverflow=1;}else{TIMER->expoverflow=0;}\
        };
#define TIMER_PAUSE(TIMER)\
        {\
                TIMER->timestamp = timer_timestamp();\
        };
#define TIMER_REMAINING(TIMER)\
        {\
                timer_time_t temp = TIMER->exptime - TIMER->timestamp;\
                TIMER->timestamp = timer_timestamp();\
                TIMER->exptime = TIMER->timestamp + temp;\
                if(TIMER->exptime < TIMER->timestamp)\
                  {TIMER->expoverflow=1;}else{TIMER->expoverflow=0;}\
        };

#define TIMER_DELETE(TIMER)\
	{\
                TIMER->state = 0;\
		TIMER->update_time = 0;\
		TIMER->callback = 0;\
	};

#ifdef DYNAMIC
#include <stdlib.h>
#else

#endif

timer_state_t timer_init( void )
{
        timers_free.count = 0;
        timers_free.head  = 0;
        timers_handled.count = 0;
        timers_handled.head  = 0;
        for(int i=0;i<TIMERS_MAX_CNT;i++)
        {
                timers_all[i].data = &timers[i];
                timers_all[i].next = 0;
                list_push_first(&timers_free,&timers_all[i]);
        }
        timers_initialized = 1;
        return TIMER_OK;
}

static int timer_compare(list_t *curr_, list_t *index_)
{
        if(curr_ == 0 || index_ == 0)
        {
                return 1;
        }
        if(timer_remaining_time((timer_t*)curr_) <= timer_remaining_time((timer_t*)index_))
        {
                return 1;
        }
        return 0;
}

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
timer_t timer_create(timer_mode_t mode_, timer_time_t time_, timer_callback_t callback_)
{
        timer_struct_t *timer = 0;
        list_t *item;
	if(timers_initialized == 0)
		return (timer_t)0;
        if(timers_free.count)
        {
                item = list_pop_first(&timers_free);
                timer = (timer_struct_t *)(item->data);
                switch(mode_)
                {
                        case TIMER_REPEAT:
                        case TIMER_ONE_SHOT:
                        case TIMER_ONE_SHOT_DELETE:
                                TIMER_CREATE(timer,mode_,time_,0,callback_);
                                break;
                        case TIMER_REPEAT_START:
                        case TIMER_ONE_SHOT_START:
                        case TIMER_ONE_SHOT_DELETE_START:
                                TIMER_CREATE(timer,mode_,time_,1,callback_);
                                TIMER_START(timer);
                                break;
                }
                if(timer->running)
                {
                  list_insert_custom(&timers_handled, item, timer_compare);
                }
                else
                {
                  list_insert_custom(&timers_not_handled, item, timer_compare);
                }
                return (timer_t)(item);
        }
	return (timer_t)0;
}

/**
 * @brief Timer delete function
 *
 * @param[in] timer_t - pointer to created timer
 *
 * @retval none
 *
 */
timer_state_t timer_delete(timer_t *timer_)
{
        timer_struct_t *timer =  (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        //if(list_push_first(&timers_free,list_pop_by_data(&timers_handled, (void*)(((list_t*)timer_)->data))))
        if(list_push_first(&timers_free,list_pop_by_data(&timers_handled, (void*)timer_) ) )
        {
          *timer_ = 0;
          TIMER_DELETE(timer);
          return TIMER_OK;
        }
        //if(list_push_first(&timers_free,list_pop_by_data(&timers_not_handled, (void*)(((list_t*)timer_)->data))))
        if(list_push_first(&timers_free,list_pop_by_data(&timers_not_handled, (void*)timer_) ) )
        {
          *timer_ = 0;
          TIMER_DELETE(timer);
          return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief Set callback to created timer
 *
 * @param[in] timer_callback_t		-	callback  function
 *
 * @retval TIMER_OK			-	timer set callback
 * @retval TIMER_ERR		-	timer is not created
 *
 */
timer_state_t timer_callback_set(timer_t timer_, timer_callback_t callback_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
        if(timer == 0)
                return TIMER_ERR;
	if(timers_initialized == 0)
		return TIMER_ERR;
        timer->callback = callback_;
	return TIMER_OK;
}

/**
 *	@brief Get callback from created timer
 *
 *	@retval timer_callback_t 	-	pointer to callback function
 *
 */
timer_callback_t timer_callback_get(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
        if(timer == 0)
                return TIMER_ERR;
	if(timers_initialized == 0)
		return TIMER_ERR;
	return timer->callback;
}

/**
 * @brief Set update time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 * @param[in] timer_time_t	-	update time
 *
 * @retval	none
 *
 */
timer_state_t timer_set_time(timer_t timer_, timer_time_t  time_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
        if(timer == 0)
                return TIMER_ERR;
	if(timers_initialized == 0)
		return TIMER_ERR;
        timer->update_time = time_;
        return TIMER_OK;
}

/**
 * @brief Get update time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_get_time(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
        if(timer == 0)
                return TIMER_ERR;
	if(timers_initialized == 0)
		return TIMER_ERR;
        return timer->update_time;
}

/**
 * @brief Get elapsed time for timer
 *
 * @param[in]   timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_elapsed_time(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return (timer_time_t)0;
        if(timer)
        {
                return timer_timestamp() - timer->timestamp;
        }
	return (timer_time_t)0;
}

/**
 * @brief Get remaining time for timer
 *
 * @param[in] timer_t		-	pointer to a timer
 *
 * @retval	timer_time_t	-	update time
 */
timer_time_t timer_remaining_time(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return (timer_time_t)0;
        if(timer)
        {
                return  timer->exptime - timer_timestamp();
        }
	return TIMER_ERR;
}


/**
 * @brief Get timer ready
 *
 * @param[in]	timer_t	-	pointer to a time
 *
 * @retval	TIMER_OK	-	timer is ready
 * @retval 	TIMER_ERR	-	timer is not ready
 *
 */
timer_state_t timer_ready(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer && timer->ready)
        {
                return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief	Get timer enable
 *
 * @param[in]	timer_t	-	pointer to a timer
 *
 * @retval	TIMER_OK	-	timer enable
 * @retval	TIMER_ERR	-	timer disable
 *
 */
timer_state_t timer_enable(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer)
        {
          timer->enabled = 1;
          return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief	Timer start
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_start(timer_t timer_)
{
        list_t *item;
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer && !(timer->running) )
        {
                timer->stop = 0;
                timer->running = 1;
                if(timer->pause)
                {
                  TIMER_REMAINING(timer);
                }
                else
                {
                  TIMER_START(timer);
                }
                item = list_pop_item(&timers_not_handled, (list_t *)(timer_));
                list_insert_custom(&timers_handled, item, timer_compare);
                return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief	Timer stop
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_reset(timer_t timer_)
{
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer)
        {
                timer->pause = 0;
                TIMER_RESET(timer);
                return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief	Timer reset time to zero
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_stop(timer_t timer_)
{
        list_t *item;
        timer_struct_t *timer;
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer_)
        {
                timer = (timer_struct_t*)(((list_t*)timer_)->data);
                timer->stop = 1;
                TIMER_RESET(timer);
                item = list_pop_item(&timers_handled, (list_t *)(timer_));
                if(item) list_insert_custom(&timers_not_handled, item, timer_compare);
                return TIMER_OK;
        }
	return TIMER_ERR;
}

/**
 * @brief	Timer pause timer
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_pause(timer_t timer_)
{
        list_t *item;
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer)
        {
                timer->running = 0;
                timer->pause = 1;
                TIMER_PAUSE(timer);
                item = list_pop_item(&timers_handled, (list_t *)(timer_));
                if(item) list_insert_custom(&timers_not_handled, item, timer_compare);
                return TIMER_OK;
        }
	return TIMER_ERR;
}


/**
 * @brief	Timer continuation timer
 *
 * @param	timer_t	-	Pointer to a timer
 *
 * @retval	none
 */
timer_state_t timer_continuation(timer_t timer_)
{
        list_t *item;
        timer_struct_t *timer = (timer_struct_t*)(((list_t*)timer_)->data);
	if(timers_initialized == 0)
		return TIMER_ERR;
        if(timer && timer->pause)
        {
                timer->stop = 0;
                timer->running = 1;
                TIMER_REMAINING(timer);
                item = list_pop_item(&timers_not_handled, (list_t *)(timer_));
                list_insert_custom(&timers_handled, item, timer_compare);
                return TIMER_OK;
        }
	return TIMER_ERR;
}



/**
 * @brief	Get global timer timestamp
 *
 * @retval	timer_time_t	-	timestamp
 */
timer_time_t timer_timestamp(void)
{
	if(timers_initialized == 0)
		return (timer_time_t)0;
	return timers_global_tick;
}

static int timer_expire(timer_struct_t *timer)
{
  if(timer->exptime <= timer_timestamp())
  {
    if(timer->expoverflow && (timer->timestamp < timer_timestamp()))
    {
      return 0;
    }
    return 1;
  }
  return 0;
}

/**
 * @brief	Check timers ready
 *
 * @retval	none
 */
static inline void timers_check_ready(void)
{
        timer_struct_t *timer;
	if(timers_initialized == 0)
		return;
        list_foreach((list_head_t*)(&timers_handled),item)
        {
                if(timer_expire((timer = (timer_struct_t *)(item->data),timer)))
                {
                        timer->ready = 1;
                }
                else break;
        }
}

/**
 *	@brief	Timer tick
 *
 *	@retval none
 *
 */
void timer_tick(void)
{
	timers_global_tick += TIMER_TICK_MS;
        timers_check_ready();
}

void timer_tick_diff(unsigned long elapsed)
{
	timers_global_tick += elapsed;
        timers_check_ready();
}


static void timer_update_info(timer_struct_t *timer)
{
        if(timer->running)
        {
                timer->ready = 0;
                timer->timestamp = timer_timestamp();
                timer->exptime = timer->timestamp + timer->update_time;
                if(timer->exptime < timer->timestamp)
                {
                        timer->expoverflow=1;
                }
                else
                {
                        timer->expoverflow=0;
                }
        }
}

/**
 *	@brief	Timer handle
 *
 *	@retval none
 *
 */
void timer_handle(void)
{
        timer_struct_t *timer;
        list_t *item;
	if(timers_initialized == 0)
		return;
        if(timers_handled.count)
        {
                while((timer = ((timer_struct_t *)((item = list_get_first(&timers_handled))->data)), timer->ready ))
                {
                        item = list_pop_first(&timers_handled);
                        switch(timer->mode)
                        {
                                case TIMER_REPEAT:
                                case TIMER_REPEAT_START:
                                        timer->callback((void*)item);
                                        timer_update_info(timer);
                                        if(timer->stop)
                                        {
                                          timer->stop = 0;
                                          timer->running = 0;
                                          list_insert_custom(&timers_not_handled, item, timer_compare);
                                        }
                                        else
                                        {
                                          list_insert_custom(&timers_handled, item, timer_compare);
                                        }
                                        break;
                                case TIMER_ONE_SHOT:
                                case TIMER_ONE_SHOT_START:
                                        timer->stop = 1;
                                        timer->callback((void*)item);
                                        timer_update_info(timer);
                                        if(timer->stop)
                                        {
                                          timer->stop = 0;
                                          timer->running = 0;
                                          list_insert_custom(&timers_not_handled, item, timer_compare);
                                        }
                                        else
                                        {
                                          list_insert_custom(&timers_handled, item, timer_compare);
                                        }
                                        break;
                                case TIMER_ONE_SHOT_DELETE:
                                case TIMER_ONE_SHOT_DELETE_START:
                                        timer->del = 1;
                                        timer->callback((void*)item);
                                        if(timer->del)
                                        {
                                          TIMER_DELETE(timer);
                                          list_insert_custom(&timers_free, item, timer_compare);
                                        }
                                        else
                                        {
                                          list_insert_custom(&timers_handled, item, timer_compare);
                                        }
                                        list_push_first(&timers_free,item);
                                        break;
                        }
                        
                }
        }
}
