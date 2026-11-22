// SPDX-License-Identifier: GPL-3.0-only
#include <inttypes.h>
#include <stdio.h>
#include <sys/socket.h>
#include "serialqueue.c"

int main(void)
{
    int fd[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fd))
        return 2;
    struct serialqueue sq = {.serial_fd = fd[0]};
    list_init(&sq.sent_queue);
    list_init(&sq.old_sent);
    debug_queue_alloc(&sq.old_sent, DEBUG_QUEUE_SENT);
    sq.pr = pollreactor_alloc(1, SQPT_NUM, &sq);
    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'n') {
            int wire; double speed;
            if (scanf("%d %lf", &wire, &speed) != 2)
                return 2;
            message_queue_free(&sq.sent_queue);
            sq.send_seq = sq.receive_seq = 17;
            sq.srtt = sq.rttvar = sq.idle_time = 0.;
            sq.rto = MIN_RTO;
            sq.rtt_sample_seq = sq.need_ack_bytes = 0;
            sq.serial_fd_type = wire == 2 ? SQT_CAN : SQT_DEBUGFILE;
            sq.bittime_adjust = wire == 1 ? 10. / speed : wire == 2 ? 1. / speed : 0.;
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, PR_NEVER);
        } else if (op == 's') {
            int len; double end;
            if (scanf("%d %lf", &len, &end) != 2)
                return 2;
            struct queue_message *m = message_alloc();
            memset(m->msg, 0, sizeof(m->msg));
            m->len = len; m->receive_time = end;
            if (!sq.rtt_sample_seq)
                sq.rtt_sample_seq = sq.send_seq;
            sq.send_seq++;
            if (list_empty(&sq.sent_queue))
                pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, end + sq.rto);
            list_add_tail(&m->node, &sq.sent_queue);
            sq.need_ack_bytes += len;
        } else if (op == 'a') {
            int delta; double now;
            if (scanf("%d %lf", &delta, &now) != 2)
                return 2;
            update_receive_seq(&sq, now, sq.receive_seq + delta);
        } else if (op == 'r') {
            int fast; double now;
            if (scanf("%d %lf", &fast, &now) != 2)
                return 2;
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, fast ? PR_NOW : now);
            double wake = retransmit_event(&sq, now);
            int total = sq.need_ack_bytes + 1;
            if (sq.serial_fd_type == SQT_CAN)
                total = DIV_ROUND_UP(total, 8) * sizeof(struct can_frame);
            while (total) {
                char discard[2048];
                int n = read(fd[1], discard, total);
                if (n <= 0) return 3;
                total -= n;
            }
            pollreactor_update_timer(sq.pr, SQPT_RETRANSMIT, wake);
        } else
            return 2;
        printf("%.17g %.17g %.17g %.17g\n", sq.rto, sq.srtt, sq.rttvar,
               pollreactor_get_timer(sq.pr, SQPT_RETRANSMIT));
    }
    message_queue_free(&sq.sent_queue);
    message_queue_free(&sq.old_sent);
    pollreactor_free(sq.pr);
    close(fd[0]);
    close(fd[1]);
    return 0;
}
