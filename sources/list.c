#include "list.h"

#define NULL 0

int list_item_set(list_t *item, void *data_)
{
  if(item == 0)
    return LIST_PUT_ERR;
  item->data = data_;
  return LIST_PUT_OK;
}

void *list_item_get(list_t *item)
{
  if(item == 0)
    return 0;
  return item->data;
}

int list_init(list_head_t *list_, list_t *item)
{
  if(list_ == 0)
    return LIST_PUT_ERR_PARAM;
  if(item)
  {
    list_->head = item;
    list_->count = 1;
    return LIST_PUT_OK_FIRST;
  }
  else
  {
    list_->head = 0;
    list_->count = 0;
  }
  return LIST_PUT_OK;
}

list_t *list_get_first(list_head_t *list_)
{
  if(list_==0)
    return 0;
  return list_->head;
}

list_t *list_get_last(list_head_t *list_)
{
  list_t *item = 0;
  item = list_->head;
  while(item->next && item->next != item)
    item=item->next;
  return item;
}

list_t *list_get_index(list_head_t *list_, unsigned int index)
{
  list_t *item;
  if(list_ == 0)
    return NULL;
  if(index == 0)
  {
    return list_->head;
  }
  item = list_->head;
  while(index>0 && item->next)
  {
    item = item->next;
    index--;
  }
  if(index)
    return NULL;
  return item;
}

unsigned int list_get_count(list_head_t *list_)
{
  unsigned int count = 0;
  list_t *item = 0;
  if(list_ == 0)
    return 0;
  if(list_->head == 0)
  {
    return 0;
  }
  item = list_->head;
  while((count++, item->next))
  {
    item = item->next;
  }
  return count;
}

int list_push_first(list_head_t *list_, list_t *item_)
{
  if(list_ == 0 || item_ == 0)
    return LIST_PUT_ERR_PARAM;
  if(list_->head == 0)
  {
    list_->head = item_;
    list_->count++;
    return LIST_PUT_OK_FIRST;
  }
  item_->next = list_->head;
  list_->head = item_;
  list_->count++;
  return LIST_PUT_OK;
}

int list_push_last(list_head_t *list_, list_t *item_)
{
  list_t *item = 0;
  if(list_ == 0 || item_ == 0)
    return LIST_PUT_ERR_PARAM;
  if(list_->head == 0)
  {
    list_->head = item_;
    list_->count++;
    return LIST_PUT_OK_FIRST;
  }
  item = list_get_last(list_);
  item->next = item_;
  list_->count++;
  return LIST_PUT_OK;
}


int list_insert_after_index(list_head_t *list_, list_t *item_, unsigned int index)
{
  list_t *item = 0;
  if(list_ == 0 || item_ == 0)
    return LIST_PUT_ERR_PARAM;
  item = list_get_index(list_,index);
  if(item)
  {
    item_->next = item->next;
    item->next = item_;
    list_->count++;
    return LIST_PUT_OK;
  }
  return LIST_PUT_ERR_PARAM;
}

int list_insert_custom(list_head_t *list_, list_t *item_, int (*compare)(list_t *item_curr, list_t *item_index))
{
  list_t *item = 0, *prev_item = 0;
  int temp;
  if(list_ == 0 || item_ == 0 || compare == 0)
    return LIST_PUT_ERR_PARAM;
  if(list_->head == 0)
  {
    if((temp = compare(item_,item)))
    {
      list_push_first(list_,item_);
      return LIST_PUT_OK;
    } else return LIST_PUT_ERR;
  }
  prev_item = 0;
  item = list_->head;
  while((temp = compare(item_,item)) == 0 && item->next != 0)
  {
    prev_item = item;
    item = item->next;
  }
  if(temp == 1)
  {
    if(prev_item)
    {
      prev_item->next = item_;
    }
    else
    {
      list_->head = item_;
    }
    item_->next = item;
  } 
  else
  {
    item_->next = item->next;
    item->next = item_;
  }
  list_->count++;
  return LIST_PUT_OK;
}


list_t *list_pop_first(list_head_t *list_)
{
  list_t *item = 0;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  item = list_->head;
  list_->head = list_->head->next;
  list_->count--;
  return (item->next = 0,item);
}

list_t *list_pop_last(list_head_t *list_)
{
  list_t *item,*item_temp;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  if(list_->head->next == 0)
  {
    item = list_->head;
    list_->head = 0;
    list_->count--;
    item->next = 0;
    return (item->next = 0,item);
  }
  item = list_->head;
  while(item->next->next)
  {
    item = item->next;
  }
  item_temp = item->next;
  item->next = 0;
  list_->count--;
  return (item_temp->next = 0,item_temp);
}

list_t *list_get_by_data(list_head_t *list_, void *data_)
{
  list_t *item;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  item = list_->head;
  while((item->next) || (item->data == data_))
  {
    item = item->next;
  }
  if(item->data == data_)
  {
    return item;
  }
  return (list_t*)0;
}

list_t *list_get_item(list_head_t *list_, list_t *item_)
{
  list_t *item;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  item = list_->head;
  while((item->next) || (item == item_))
  {
    item = item->next;
  }
  if(item == item_)
  {
    return (item);
  }
  return (list_t*)0;
}

list_t *list_pop_by_data(list_head_t *list_, void *data_)
{
  list_t *item,*prev_item;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  item = list_->head;
  while(item->next)
  {
    prev_item = item;
    item = item->next;
  }
  if(item->data == data_)
  {
    prev_item->next = item->next;
    return (item->next = 0,item);
  }
  return (list_t*)0;
}


list_t *list_pop_item(list_head_t *list_, list_t *item_)
{
  list_t *item,*prev_item;
  if(list_ == 0)
    return NULL;
  if(list_->head == 0)
  {
    return NULL;
  }
  if(list_->head == item_)
  {
    list_->head = list_->head->next;
    list_->count--;
    return item_;
  }
  prev_item = 0;
  item = list_->head;
  while(item->next && item != item_)
  {
    prev_item = item;
    item = item->next;
  }
  if(item == item_)
  {
    prev_item->next = item->next;
    list_->count--;
    return (item->next = 0,item);
  }
  return NULL;
}