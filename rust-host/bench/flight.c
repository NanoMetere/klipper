// SPDX-License-Identifier: GPL-3.0-only
// Exercise pinned stock receive and retransmit routines in an isolated pipe.
#include <inttypes.h>
#include <stdio.h>
#include "serialqueue.c"

static int
make_frame(uint8_t *buf, int seq, int payload, int byte)
{
    int n = payload + MESSAGE_MIN;
    buf[0] = n;
    buf[1] = MESSAGE_DEST | (seq & MESSAGE_SEQ_MASK);
    memset(buf + 2, byte, payload);
    uint16_t crc = msgblock_crc16_ccitt(buf, n - 3);
    buf[n - 3] = crc >> 8;
    buf[n - 2] = crc & 255;
    buf[n - 1] = MESSAGE_SYNC;
    return n;
}

static void
hex(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        printf("%02x", buf[i]);
    putchar('\n');
}

int
main(void)
{
    struct serialqueue sq = {0};
    int fd[2];
    if (pipe(fd))
        return 2;
    sq.serial_fd = fd[1];
    sq.serial_fd_type = SQT_DEBUGFILE; // Pipe writes without UART tcflush.
    sq.send_seq = sq.receive_seq = sq.last_ack_seq = 14;
    sq.rto = MIN_RTO;
    sq.pr = pollreactor_alloc(1, SQPT_NUM, &sq);
    list_init(&sq.sent_queue);
    list_init(&sq.old_sent);
    debug_queue_alloc(&sq.old_sent, DEBUG_QUEUE_SENT);
    list_init(&sq.notify_queue);
    list_init(&sq.fast_readers);
    list_init(&sq.receiver.queue);
    pthread_mutex_init(&sq.lock, NULL);
    pthread_mutex_init(&sq.receiver.lock, NULL);

    char op;
    unsigned step = 1;
    while (scanf(" %c", &op) == 1) {
        if (op == 's') {
            int payload, byte, window;
            if (scanf("%d %d %d", &payload, &byte, &window) != 3
                || payload < 1 || payload > MESSAGE_PAYLOAD_MAX)
                return 2;
            int prior = sq.last_ack_seq < sq.receive_seq ? sq.last_ack_bytes : 0;
            if (sq.send_seq - sq.receive_seq >= MAX_PENDING_BLOCKS
                || (sq.send_seq > sq.receive_seq && window
                    && sq.need_ack_bytes + prior + MESSAGE_MAX > window)) {
                puts("blocked");
                continue;
            }
            uint8_t data[MESSAGE_MAX];
            int len = make_frame(data, sq.send_seq, payload, byte);
            struct queue_message *qm = message_fill(data, len);
            qm->receive_time = step;
            list_add_tail(&qm->node, &sq.sent_queue);
            sq.send_seq++;
            sq.need_ack_bytes += len;
            hex(data, len);
        } else if (op == 'a') {
            int seq, payload;
            if (scanf("%d %d", &seq, &payload) != 2 || payload < 0 || payload > 1)
                return 2;
            uint64_t invalid = sq.bytes_invalid;
            int len = make_frame(sq.input_buf, seq, payload, 37);
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, PR_NEVER);
            handle_message(&sq, step, len);
            if (invalid != sq.bytes_invalid)
                puts("invalid");
            else {
                int prior = sq.last_ack_seq < sq.receive_seq ? sq.last_ack_bytes : 0;
                int fast = pollreactor_get_timer(sq.pr, SQPT_RETRANSMIT) == PR_NOW;
                int timed = 0;
                if (payload) {
                    struct queue_message *qm = list_first_entry(
                        &sq.receiver.queue, struct queue_message, node);
                    timed = qm->sent_time != 0.;
                    message_queue_free(&sq.receiver.queue);
                }
                printf("%" PRIu64 " %" PRIu64 " %d %d %d %d\n",
                       sq.receive_seq, sq.send_seq - sq.receive_seq,
                       sq.need_ack_bytes, prior, fast, timed);
                if (fast && (step & 1)) {
                    double wake = retransmit_event(&sq, step);
                    uint8_t data[1 + MAX_PENDING_BLOCKS * MESSAGE_MAX];
                    int n = read(fd[0], data, sizeof(data));
                    if (n != sq.need_ack_bytes + 1)
                        return 3;
                    hex(data, n);
                    pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, wake);
                }
            }
        } else if (op == 't') {
            if (list_empty(&sq.sent_queue)) {
                puts("empty");
                continue;
            }
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, step);
            double wake = retransmit_event(&sq, step);
            uint8_t data[1 + MAX_PENDING_BLOCKS * MESSAGE_MAX];
            int n = read(fd[0], data, sizeof(data));
            if (n != sq.need_ack_bytes + 1)
                return 3;
            hex(data, n);
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, wake);
        } else
            return 2;
        step++;
    }
    message_queue_free(&sq.sent_queue);
    message_queue_free(&sq.old_sent);
    pollreactor_free(sq.pr);
    pthread_mutex_destroy(&sq.lock);
    pthread_mutex_destroy(&sq.receiver.lock);
    close(fd[0]);
    close(fd[1]);
    return 0;
}
