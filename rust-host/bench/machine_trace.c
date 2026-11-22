// SPDX-License-Identifier: GPL-3.0-only
// Run stock trapq + itersolve + stepcompress entirely in memory.
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "itersolve.h"
#include "msgblock.h"
#include "stepcompress.h"
#include "trapq.h"

struct stepper_kinematics *cartesian_stepper_alloc(char axis);
struct stepper_kinematics *corexy_stepper_alloc(char axis);

int main(void)
{
    struct trapq *q = trapq_alloc();
    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'm') {
            double at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &at, &a, &c, &d, &x, &y, &z, &rx, &ry, &rz, &sv, &cv, &accel) != 13)
                return 2;
            trapq_append(q, at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel);
        } else if (op == 's') {
            double xpos, ypos, zpos, step, last, hz;
            unsigned err, invert;
            char axis;
            if (scanf(" %c %lf %lf %lf %lf %lf %lf %u %u", &axis, &xpos, &ypos,
                      &zpos, &step, &last, &hz, &err, &invert) != 9)
                return 2;
            struct stepper_kinematics *sk = axis == '+' || axis == '-'
                ? corexy_stepper_alloc(axis) : cartesian_stepper_alloc(axis);
            struct list_head messages;
            list_init(&messages);
            struct stepcompress *sc = stepcompress_alloc(&messages);
            stepcompress_fill(sc, 1, err, 3, 4);
            stepcompress_set_invert_sdir(sc, invert);
            stepcompress_set_time(sc, 0., hz);
            itersolve_set_trapq(sk, q, step);
            itersolve_set_position(sk, xpos, ypos, zpos);
            trapq_check_sentinels(q);
            if (itersolve_generate_steps(sk, sc, last) || stepcompress_flush(sc, UINT64_MAX))
                return 3;
            struct queue_message *m;
            uint64_t tick = 0;
            list_for_each_entry(m, &messages, node) {
                uint32_t fields[5];
                uint8_t framed[MESSAGE_MAX] = {0};
                memcpy(framed + MESSAGE_HEADER_SIZE, m->msg, m->len);
                int len = m->len + MESSAGE_MIN;
                if (!msgblock_decode(fields, 3, framed, len) && fields[0] == 4) {
                    printf("d %u\n", fields[2]);
                } else if (!msgblock_decode(fields, 5, framed, len) && fields[0] == 3) {
                    int64_t count = fields[3], add = (int32_t)fields[4];
                    tick += fields[2] * count + add * count * (count - 1) / 2;
                    printf("s %u %" PRId64 " %" PRId64 " %" PRIu64 "\n",
                           fields[2], count, add, tick);
                } else
                    return 4;
            }
            puts("end");
            free(sk);
            stepcompress_free(sc);
            message_queue_free(&messages);
        } else
            return 2;
    }
    trapq_free(q);
    return 0;
}
