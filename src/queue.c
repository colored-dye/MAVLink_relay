#include "queue.h"
#include "mavlink_types.h"
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <string.h>

void queue_init(queue_t *q) {
    memset(q->raw_queue, -1, MAX_QUEUE_SIZE * sizeof(mavlink_message_t));
    q->head = 0;
    q->size = 0;
}

int queue_full(queue_t *q) {
    return q->size == MAX_QUEUE_SIZE;
}

int queue_empty(queue_t *q) {
    return !q->size;
}

int enqueue(queue_t *q, mavlink_message_t msg) {
    int ret = 0;
    if (queue_full(q)) {
        fprintf(stderr, "Cannot enqueue");
        ret = -1;
    } else {
        uint32_t index = (q->head + q->size) % MAX_QUEUE_SIZE;
        q->raw_queue[index] = msg;
        q->size++;
    }
    return ret;
}

int dequeue(queue_t *q, mavlink_message_t *msg) {
    int ret = 0;

    if (queue_empty(q)) {
        fprintf(stderr, "Cannot dequeue");
        ret = -1;
    } else {
        *msg = q->raw_queue[q->head];
        q->head = (q->head + 1) % MAX_QUEUE_SIZE;
        q->size--;
    }

    return ret;
}
