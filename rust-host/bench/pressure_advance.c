// SPDX-License-Identifier: GPL-3.0-only
#include <stddef.h>
#include <stdio.h>
#include "trapq.h"
#include "itersolve.h"

struct stepper_kinematics *extruder_stepper_alloc(void);
void extruder_stepper_free(struct stepper_kinematics *sk);
void extruder_set_pressure_advance(struct stepper_kinematics *sk, double print_time,
                                    double pressure_advance, double smooth_time);

int main(void)
{
    char op;
    struct trapq *q = NULL;
    struct stepper_kinematics *sk = NULL;
    while (scanf(" %c", &op) == 1) {
        if (op == 'r') {
            if (q) trapq_free(q);
            if (sk) extruder_stepper_free(sk);
            q = trapq_alloc();
            sk = extruder_stepper_alloc();
        } else if (op == 'g') {
            double t, k, smooth;
            if (scanf("%lf %lf %lf", &t, &k, &smooth) != 3) return 2;
            extruder_set_pressure_advance(sk, t, k, smooth);
        } else if (op == 'm') {
            double at, a, c, d, x, sv, cv, accel, y;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf", &at, &a, &c, &d,
                      &x, &sv, &cv, &accel, &y) != 9) return 2;
            trapq_append(q, at, a, c, d, x, 0., 0., 1., y, 0., sv, cv, accel);
        } else if (op == 'p') {
            double t;
            if (scanf("%lf", &t) != 1) return 2;
            trapq_check_sentinels(q);
            struct move *head = list_first_entry(&q->moves, struct move, node);
            struct move *tail = list_last_entry(&q->moves, struct move, node);
            struct move *m = list_next_entry(head, node);
            while (m != tail) {
                struct move *n = list_next_entry(m, node);
                if (n->print_time > t) break;
                m = n;
            }
            printf("%.17g\n", sk->calc_position_cb(sk, m, t - m->print_time));
        } else return 2;
    }
    if (q) trapq_free(q);
    if (sk) extruder_stepper_free(sk);
    return 0;
}
