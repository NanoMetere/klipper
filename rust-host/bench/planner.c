// SPDX-License-Identifier: GPL-3.0-only
// Offline harness: links the unmodified pinned scheduler internals, never starts
// a reactor thread or opens a serial/CAN device.
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "serialqueue.c"

static void
hex(const uint8_t *data, int n)
{
    for (int i = 0; i < n; ++i)
        printf("%02x", data[i]);
    putchar('\n');
}

int
main(void)
{
    enum { LANES = 8 };
    struct serialqueue sq = {0};
    struct command_queue *lane[LANES];
    list_init(&sq.transmit_requests.upcoming_queues);
    list_init(&sq.ready_queues);
    list_init(&sq.sent_queue);
    list_init(&sq.notify_queue);
    sq.transmit_requests.min_release_clock = MAX_CLOCK;
    pthread_mutex_init(&sq.transmit_requests.lock, NULL);
    pthread_mutex_init(&sq.lock, NULL);
    sq.pr = pollreactor_alloc(1, SQPT_NUM, &sq);
    for (int q = 0; q < LANES; ++q)
        lane[q] = serialqueue_alloc_commandqueue();

    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'p') {
            int q, len, value;
            uint64_t release, priority;
            if (scanf("%d %" SCNu64 " %" SCNu64 " %d %d", &q, &release,
                      &priority, &len, &value) != 5
                || q < 0 || q >= LANES || len < 1 || len > MESSAGE_PAYLOAD_MAX)
                return 2;
            uint8_t payload[MESSAGE_PAYLOAD_MAX];
            memset(payload, value, len);
            // No worker or kick pipe in this single-threaded harness.
            sq.transmit_requests.need_kick_clock = 0;
            serialqueue_send(&sq, lane[q], payload, len, release, priority, 0);
        } else if (op == 'd') {
            int wire, pending, flight, outstanding, prior, window;
            double baud, hz, anchor, now, idle;
            uint64_t ticks;
            if (scanf("%d %lf %lf %lf %" SCNu64 " %lf %lf %d %d %d %d %d",
                      &wire, &baud, &hz, &anchor, &ticks, &now, &idle,
                      &pending, &flight, &outstanding, &prior, &window) != 12)
                return 2;
            sq.serial_fd_type = wire == 2 ? SQT_CAN : SQT_UART;
            if (wire)
                serialqueue_set_wire_frequency(&sq, baud);
            else
                sq.bittime_adjust = 0.;
            clock_fill(&sq.ce, hz, anchor, ticks);
            sq.idle_time = idle;
            sq.send_seq = 1024 + flight;
            sq.receive_seq = 1024;
            sq.last_ack_seq = 1024 - !!prior;
            sq.need_ack_bytes = outstanding;
            sq.last_ack_bytes = prior;
            sq.receive_window = window;
            double decision = check_send_command(&sq, pending, now);
            if (decision == PR_NOW) {
                uint8_t data[MESSAGE_MAX];
                int len = build_and_send_command(&sq, data, pending, now);
                fputs("send ", stdout);
                hex(data, len);
                message_queue_free(&sq.sent_queue);
            } else if (flight >= MAX_PENDING_BLOCKS
                       || (flight && window && outstanding + prior + MESSAGE_MAX > window)) {
                printf("ack %.17g\n", decision);
            } else if (pending && hz) {
                printf("drain %.17g\n", decision);
            } else if (!sq.ready_bytes && !sq.transmit_requests.upcoming_bytes) {
                puts("idle");
            } else if (!hz && decision == PR_NEVER) {
                puts("idle");
            } else {
                printf("sleep %.17g\n", decision);
            }
        } else {
            return 2;
        }
    }
    for (int q = 0; q < LANES; ++q) {
        message_queue_free(&lane[q]->ready.msg_queue);
        message_queue_free(&lane[q]->upcoming.msg_queue);
        serialqueue_free_commandqueue(lane[q]);
    }
    pollreactor_free(sq.pr);
    pthread_mutex_destroy(&sq.lock);
    pthread_mutex_destroy(&sq.transmit_requests.lock);
    return 0;
}
