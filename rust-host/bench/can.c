// SPDX-License-Identifier: GPL-3.0-only
// Stock packetization only; an AF_UNIX datagram socketpair is not SocketCAN.
#include <stdio.h>
#include <sys/socket.h>
#include "serialqueue.c"

struct queue {
    uint8_t data[80], need_sync;
    int len;
};

static void hex(const uint8_t *data, int n)
{
    for (int i = 0; i < n; i++)
        printf("%02x", data[i]);
    putchar('\n');
}

int main(void)
{
    enum { NODES = 6 };
    struct queue rx[NODES] = {0};
    int fd[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, fd))
        return 2;
    struct serialqueue sq = {.serial_fd = fd[0], .serial_fd_type = SQT_CAN};
    char op;
    unsigned which, size, value;
    while (scanf(" %c %u %u", &op, &which, &size) == 3) {
        uint8_t data[769];
        if (size > sizeof(data) || (op == 'r' && size > 8))
            return 3;
        for (unsigned i = 0; i < size; i++) {
            if (scanf("%u", &value) != 1 || value > 255)
                return 2;
            data[i] = value;
        }
        if (op == 't') {
            sq.client_id = 256 + which * 2;
            do_write(&sq, data, size);
            for (unsigned i = 0; i < size; i += 8) {
                struct can_frame cf;
                if (read(fd[1], &cf, sizeof(cf)) != sizeof(cf))
                    return 2;
                printf("tx %u ", cf.can_id);
                hex(cf.data, cf.can_dlc);
            }
        } else if (op == 'r') {
            if (which < 265 || which > 275 || !(which & 1)) {
                puts("filtered");
                continue;
            }
            struct queue *buf = &rx[(which - 265) / 2];
            memcpy(buf->data + buf->len, data, size);
            buf->len += size;
            int discard = 0, frames = 0;
            for (;;) {
                int n = msgblock_check(&buf->need_sync, buf->data, buf->len);
                if (!n)
                    break;
                if (n < 0) {
                    n = -n;
                    discard += n;
                } else {
                    fputs("rx ", stdout);
                    hex(buf->data, n);
                    frames++;
                }
                memmove(buf->data, buf->data + n, buf->len - n);
                buf->len -= n;
            }
            printf("rx-count %d %d\n", frames, discard);
        } else
            return 2;
    }
    close(fd[0]);
    close(fd[1]);
    return 0;
}
