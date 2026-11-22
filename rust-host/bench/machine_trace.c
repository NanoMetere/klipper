// SPDX-License-Identifier: GPL-3.0-only
// Run stock trapq + itersolve + stepcompress entirely in memory.
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "itersolve.h"
#include "msgblock.h"
#include "stepcompress.h"
#include "trapq.h"

struct stepper_kinematics *cartesian_stepper_alloc(char axis);
struct stepper_kinematics *corexy_stepper_alloc(char axis);
struct stepper_kinematics *input_shaper_alloc(void);
int input_shaper_set_sk(struct stepper_kinematics *, struct stepper_kinematics *);
int input_shaper_set_shaper_params(struct stepper_kinematics *, char, int, double[], double[]);

struct stepper_kinematics *extruder_stepper_alloc(void);
void extruder_stepper_free(struct stepper_kinematics *sk);
void extruder_set_pressure_advance(struct stepper_kinematics *sk, double print_time,
                                    double pressure_advance, double smooth_time);

uint64_t machine_trace_cpu_ns(void)
{
    struct timespec c;
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &c))
        abort();
    return c.tv_sec * 1000000000ULL + c.tv_nsec;
}


#ifndef MACHINE_TRACE_CLOCK_ONLY
int main(void)
{
    struct trapq *q = trapq_alloc(), *eq = trapq_alloc();
    char op;
    int n_params = 0;
    uint64_t processing = 0;
    double params[8][3];
    struct { int n; double a[10], t[10]; } shapes[3] = {0};
    int shaped = 0;
    while (scanf(" %c", &op) == 1) {
        if (op == 'm' || op == 'e') {
            double at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &at, &a, &c, &d, &x, &y, &z, &rx, &ry, &rz, &sv, &cv, &accel) != 13)
                return 2;
            uint64_t started = machine_trace_cpu_ns();
            trapq_append(op == 'm' ? q : eq, at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel);
            processing += machine_trace_cpu_ns() - started;
        } else if (op == 'g') {
            if (n_params >= 8 || scanf("%lf %lf %lf", &params[n_params][0],
                                        &params[n_params][1], &params[n_params][2]) != 3)
                return 2;
            n_params++;
        } else if (op == 'h') {
            char axis;
            int n;
            if (scanf(" %c %d", &axis, &n) != 2 || axis < 'x' || axis > 'z' || n < 0 || n > 10)
                return 2;
            int k = axis - 'x';
            for (int j = 0; j < n; j++)
                if (scanf("%lf %lf", &shapes[k].t[j], &shapes[k].a[j]) != 2)
                    return 2;
            shapes[k].n = n;
            shaped = 1;
        } else if (op == 's') {
            double xpos, ypos, zpos, step, last, hz;
            unsigned err, invert;
            char axis;
            if (scanf(" %c %lf %lf %lf %lf %lf %lf %u %u", &axis, &xpos, &ypos,
                      &zpos, &step, &last, &hz, &err, &invert) != 9)
                return 2;
            uint64_t started = machine_trace_cpu_ns();
            struct stepper_kinematics *sk = axis == 'e' ? extruder_stepper_alloc()
                : axis == '+' || axis == '-' ? corexy_stepper_alloc(axis) : cartesian_stepper_alloc(axis);
            struct stepper_kinematics *original = sk;
            if (shaped && axis != 'e') {
                sk = input_shaper_alloc();
                if (input_shaper_set_sk(sk, original))
                    return 5;
                for (char a = 'x'; a <= 'z'; a++) {
                    int k = a - 'x';
                    if (input_shaper_set_shaper_params(sk, a, shapes[k].n, shapes[k].a, shapes[k].t))
                        return 5;
                }
            }
            struct trapq *trajectory = axis == 'e' ? eq : q;
            struct list_head messages;
            list_init(&messages);
            struct stepcompress *sc = stepcompress_alloc(&messages);
            stepcompress_fill(sc, 1, err, 3, 4);
            stepcompress_set_invert_sdir(sc, invert);
            stepcompress_set_time(sc, 0., hz);
            itersolve_set_trapq(sk, trajectory, step);
            itersolve_set_position(sk, xpos, ypos, zpos);
            if (axis == 'e')
                for (int i = 0; i < n_params; i++)
                    extruder_set_pressure_advance(sk, params[i][0], params[i][1], params[i][2]);
            trapq_check_sentinels(trajectory);
            if (itersolve_generate_steps(sk, sc, last) || stepcompress_flush(sc, UINT64_MAX))
                return 3;
            processing += machine_trace_cpu_ns() - started;
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
            if (axis == 'e') extruder_stepper_free(sk);
            else {
                if (original != sk)
                    free(original);
                free(sk);
            }
            stepcompress_free(sc);
            message_queue_free(&messages);
        } else
            return 2;
    }
    trapq_free(q);
    trapq_free(eq);
    fprintf(stderr, "TIMED_NS %" PRIu64 "\n", processing);
    return 0;
}
#endif
