// SPDX-License-Identifier: GPL-3.0-only
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "msgblock.h"
#include "stepcompress.h"

int main(void)
{
    char op;
    struct list_head messages;
    struct stepcompress *sc = NULL;
    uint64_t base = 0;
    list_init(&messages);
    while (scanf(" %c", &op) == 1) {
        if (op == 'r') {
            unsigned error, invert;
            double origin, hz;
            if (scanf("%" SCNu64 " %lf %lf %u %u", &base, &origin, &hz, &error, &invert) != 5)
                return 2;
            sc = stepcompress_alloc(&messages);
            stepcompress_fill(sc, 1, error, 3, 4);
            stepcompress_set_invert_sdir(sc, invert);
            stepcompress_set_time(sc, origin, hz);
            if (stepcompress_reset(sc, base))
                return 3;
        } else if (op == 'a') {
            unsigned dir;
            double at, dt;
            if (scanf("%u %lf %lf", &dir, &at, &dt) != 3 || !sc)
                return 2;
            if (stepcompress_append(sc, dir, at, dt))
                return 3;
        } else if (op == 'c') {
            if (!sc || stepcompress_commit(sc))
                return 3;
        } else if (op == 'f') {
            if (!sc || stepcompress_flush(sc, UINT64_MAX))
                return 3;
            struct queue_message *m;
            list_for_each_entry(m, &messages, node) {
                uint32_t parts[5];
                uint8_t framed[MESSAGE_MAX] = {0};
                memcpy(framed + MESSAGE_HEADER_SIZE, m->msg, m->len);
                int len = m->len + MESSAGE_MIN;
                if (!msgblock_decode(parts, 3, framed, len) && parts[0] == 4) {
                    printf("d %u\n", parts[2]);
                } else if (!msgblock_decode(parts, 5, framed, len) && parts[0] == 3) {
                    int64_t count = parts[3], add = (int32_t)parts[4];
                    base += parts[2] * count + add * count * (count - 1) / 2;
                    printf("s %u %" PRId64 " %" PRId64 " %" PRIu64 "\n",
                           parts[2], count, add, base);
                } else
                    return 4;
            }
            message_queue_free(&messages);
            stepcompress_free(sc);
            sc = NULL;
            puts("done");
        } else
            return 2;
    }
    return 0;
}
