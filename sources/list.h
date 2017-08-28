/**
  ******************************************************************************
  * @file       list.h
  * @author     Dmitry Vakhrushev (vdv.18@mail.ru)
  * @version    V1.0
  * @date       12 дек. 2014 г.
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
#ifndef LIST_H_
#define LIST_H_

#define LIST_PUT_OK             1
#define LIST_PUT_OK_FIRST       1
#define LIST_PUT_ERR            0
#define LIST_PUT_ERR_PARAM      0

typedef struct list_s {
  struct list_s *next;
  void *data;
} list_t;

typedef struct list_head_s {
  struct list_s *head;
  unsigned int count;
} list_head_t;


int list_item_set(list_t *item, void *data_);
void *list_item_get(list_t *item);

int list_init(list_head_t *list_, list_t *item);

list_t *list_get_first(list_head_t *list_);
list_t *list_get_last(list_head_t *list_);
list_t *list_get_index(list_head_t *list_, unsigned int index);
list_t *list_get_by_data(list_head_t *list_, void *data_);
list_t *list_pop_item(list_head_t *list_, list_t *item_);

unsigned int list_get_count(list_head_t *list_);

int list_push_first(list_head_t *list_, list_t *item_);
int list_push_last(list_head_t *list_, list_t *item_);
int list_insert_after_index(list_head_t *list_, list_t *item_, unsigned int index);
int list_insert_custom(list_head_t *list_, list_t *item_, int (*compare)(list_t *item_curr, list_t *item_index));

list_t *list_pop_first(list_head_t *list_);
list_t *list_pop_last(list_head_t *list_);
list_t *list_pop_by_data(list_head_t *list_, void *data_);
list_t *list_pop_item(list_head_t *list_, list_t *item_);

#define list_foreach(LIST,ITEM) for(list_t *ITEM=(LIST)->head;ITEM!=0;ITEM=ITEM->next)

#endif /* LIST_H_ */
