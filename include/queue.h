#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <mavlink_types.h>

#define MAX_QUEUE_SIZE 1

typedef struct queue_ {
    mavlink_message_t raw_queue[MAX_QUEUE_SIZE];
    uint32_t head;
    uint32_t size;
} queue_t;

void queue_init(queue_t *q);
int queue_full(queue_t *);
int queue_empty(queue_t *);
int enqueue(queue_t *, mavlink_message_t);
int dequeue(queue_t *, mavlink_message_t *);

#endif
