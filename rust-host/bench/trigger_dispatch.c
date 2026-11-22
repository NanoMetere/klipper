// SPDX-License-Identifier: GPL-3.0-only
// Real stock trigger dispatch with in-memory serial queue callbacks.
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serialqueue.h"

struct trdispatch;
struct trdispatch_mcu;
struct trdispatch *trdispatch_alloc(void);
void trdispatch_start(struct trdispatch *, uint32_t);
void trdispatch_stop(struct trdispatch *);
struct trdispatch_mcu *trdispatch_mcu_alloc(struct trdispatch *, struct serialqueue *,
    struct command_queue *, uint32_t, uint32_t, uint32_t, uint32_t);
void trdispatch_mcu_setup(struct trdispatch_mcu *, uint64_t, uint64_t, uint64_t, uint64_t);

struct serialqueue {
    struct clock_estimate ce;
    struct fastreader *fr;
    int peer;
};

void serialqueue_add_fastreader(struct serialqueue *q, struct fastreader *reader)
{
    q->fr = reader;
}

void serialqueue_rm_fastreader(struct serialqueue *q, struct fastreader *reader)
{
    if (q->fr != reader)
        abort();
    q->fr = NULL;
}

void serialqueue_get_clock_est(struct serialqueue *q, struct clock_estimate *ce)
{
    *ce = q->ce;
}

static void frame(struct queue_message *q, uint8_t out[MESSAGE_MAX])
{
    memset(out, 0, MESSAGE_MAX);
    memcpy(out + MESSAGE_HEADER_SIZE, q->msg, q->len);
}

void serialqueue_send_one(struct serialqueue *q, struct command_queue *unused,
                          struct queue_message *msg)
{
    uint8_t wire[MESSAGE_MAX];
    uint32_t values[3];
    frame(msg, wire);
    if (msgblock_decode(values, 3, wire, msg->len + MESSAGE_MIN))
        abort();
    switch (values[0]) {
    case 5:
        printf("E:%d:%u:%" PRIu64 " ", q->peer, values[1], msg->req_clock);
        if (values[2] != (uint32_t)msg->req_clock)
            abort();
        break;
    case 6:
        printf("T:%d:%u:%u ", q->peer, values[1], values[2]);
        break;
    default:
        abort();
    }
    free(msg);
}

int main(void)
{
    struct trdispatch *d = NULL;
    struct serialqueue queues[32];
    struct trdispatch_mcu *nodes[32];
    int n = 0, started = 0;
    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'P') {
            if (d) {
                trdispatch_stop(d);
                for (int i = 0; i < n; i++)
                    free(nodes[i]);
                free(d);
            }
            d = trdispatch_alloc();
            n = started = 0;
        } else if (op == 'M') {
            unsigned oid, timeout, extend;
            uint64_t start, expires, anchor;
            double hz, at;
            if (n >= 32 || scanf("%u %" SCNu64 " %" SCNu64 " %u %u %lf %lf %" SCNu64,
                    &oid, &start, &expires, &timeout, &extend, &hz, &at, &anchor) != 8)
                return 2;
            struct serialqueue *q = &queues[n];
            q->peer = n;
            clock_fill(&q->ce, hz, at, anchor);
            nodes[n] = trdispatch_mcu_alloc(d, q, NULL, oid, 5, 6, 7);
            trdispatch_mcu_setup(nodes[n++], start, expires, timeout, extend);
        } else if (op == 'S') {
            int peer;
            unsigned oid, active, reason, clock;
            double host, hz, at;
            uint64_t anchor;
            if (scanf("%d %u %u %u %u %lf %lf %lf %" SCNu64, &peer, &oid, &active, &reason,
                       &clock, &host, &hz, &at, &anchor) != 9 || peer < 0 || peer >= n)
                return 2;
            if (!started) {
                trdispatch_start(d, 2);
                started = 1;
            }
            struct serialqueue *q = &queues[peer];
            clock_fill(&q->ce, hz, at, anchor);
            uint32_t fields[] = {7, oid, active, reason, clock};
            struct queue_message *msg = message_alloc_and_encode(fields, 5);
            uint8_t wire[MESSAGE_MAX];
            frame(msg, wire);
            q->fr->func(q->fr, host, wire, msg->len + MESSAGE_MIN);
            free(msg);
            puts("");
        } else {
            return 2;
        }
    }
    if (d) {
        trdispatch_stop(d);
        for (int i = 0; i < n; i++)
            free(nodes[i]);
        free(d);
    }
    return 0;
}
