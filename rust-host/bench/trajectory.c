// SPDX-License-Identifier: GPL-3.0-only
#include <stdio.h>
#include <stddef.h>
#include "trapq.h"

int
main(void)
{
    struct trapq *q = trapq_alloc();
    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'a') {
            double at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &at, &a, &c, &d, &x, &y, &z, &rx, &ry, &rz, &sv, &cv, &accel) != 13)
                return 2;
            trapq_append(q, at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel);
        } else if (op == 'p') {
            double at;
            if (scanf("%lf", &at) != 1)
                return 2;
            struct move *first = list_first_entry(&q->moves, struct move, node);
            struct move *last = list_last_entry(&q->moves, struct move, node);
            struct move *m = list_next_entry(first, node);
            if (m == last) {
                puts("empty");
                continue;
            }
            for (;;) {
                struct move *next = list_next_entry(m, node);
                if (next == last || next->print_time > at)
                    break;
                m = next;
            }
            double offset = at - m->print_time;
            offset = offset < 0. ? 0. : offset > m->move_t ? m->move_t : offset;
            struct coord xyz = move_get_coord(m, offset);
            printf("p %.17g %.17g %.17g\n", xyz.x, xyz.y, xyz.z);
        } else if (op == 'd') {
            struct move *move, *head = list_first_entry(&q->moves, struct move, node);
            struct move *tail = list_last_entry(&q->moves, struct move, node);
            list_for_each_entry(move, &q->moves, node) {
                if (move == head || move == tail)
                    continue;
                printf("s %.17g %.17g %.17g %.17g %.17g", move->print_time,
                       move->move_t, move->start_v, move->half_accel, move->start_pos.x);
                printf(" %.17g %.17g %.17g %.17g %.17g\n", move->start_pos.y, move->start_pos.z,
                       move->axes_r.x, move->axes_r.y, move->axes_r.z);
            }
        } else
            return 2;
    }
    trapq_free(q);
    return 0;
}
