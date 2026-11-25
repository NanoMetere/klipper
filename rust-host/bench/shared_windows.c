// SPDX-License-Identifier: GPL-3.0-only
// Two pinned stock C trap queues and one stepper, with an inert event sink.
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "itersolve.h"
#include "trapq.h"

#ifdef WIRE
#include <inttypes.h>
#include <stddef.h>
#include <string.h>
#include "msgblock.h"
#include "stepcompress.h"
#else
struct stepcompress { int dir; };
int stepcompress_get_step_dir(struct stepcompress *s) { return s->dir; }
int stepcompress_commit(struct stepcompress *s) { puts("commit"); return 0; }
int stepcompress_append(struct stepcompress *s, int dir, double start, double offset)
{
    s->dir = dir;
    printf("step %d %.17g %.17g\n", dir, start, offset);
    return 0;
}
#endif
struct stepper_kinematics *extruder_stepper_alloc(void);
void extruder_stepper_free(struct stepper_kinematics *sk);
void extruder_set_pressure_advance(struct stepper_kinematics *, double, double, double);

int main(int argc, char *argv[])
{
    const double dist = 22.17 / 3200.;
#ifdef WIRE
    struct list_head messages;
    list_init(&messages);
    struct stepcompress *sink = stepcompress_alloc(&messages);
    unsigned error = argc >= 3 ? strtoul(argv[2], NULL, 10) : 0;
    stepcompress_fill(sink, 1, error, 3, 4);
    double hz = argc >= 2 ? strtod(argv[1], NULL) : 20000000.;
    if (!isfinite(hz) || !(hz > 0.))
        return 2;
    stepcompress_set_time(sink, 0., hz);
#else
    (void)argc;
    (void)argv;
    struct stepcompress raw = {0};
    struct stepcompress *sink = &raw;
#endif
    struct trapq *queue[2] = {trapq_alloc(), trapq_alloc()};
    struct stepper_kinematics *sk = extruder_stepper_alloc();
    char op;
    double prior_cutoff = 0.;
    unsigned cutoff_count = 0;
    while (scanf(" %c", &op) == 1) {
        if (op == 'a') {
            int q, pa;
            double at, ta, tc, td, x, sv, cv, acc;
            if (scanf(" %d %d %lf %lf %lf %lf %lf %lf %lf %lf",
                      &q, &pa, &at, &ta, &tc, &td, &x, &sv, &cv, &acc) != 10 || q < 0 || q > 1)
                return 2;
            trapq_append(queue[q], at, ta, tc, td, x, 0., 0., 1., pa, 0., sv, cv, acc);
        } else if (op == 'p') {
            double at, amount;
            if (scanf("%lf %lf", &at, &amount) != 2)
                return 2;
            extruder_set_pressure_advance(sk, at, amount, .04);
        } else if (op == 'b') {
            int q;
            double x;
            if (scanf("%d %lf", &q, &x) != 2 || q < -1 || q > 1)
                return 2;
            if (q >= 0)
                itersolve_set_position(sk, x, 0., 0.);
            itersolve_set_trapq(sk, q >= 0 ? queue[q] : NULL, dist);
            printf("bind %d %.17g\n", q, sk->commanded_pos);
        } else if (op == 't') {
            int q;
            double before, clear;
            if (scanf("%d %lf %lf", &q, &before, &clear) != 3 || q < 0 || q > 1)
                return 2;
            trapq_finalize_moves(queue[q], before, clear);
        } else if (op == 'w') {
            double cutoff;
            if (++cutoff_count > 512 || scanf("%lf", &cutoff) != 1
                || !isfinite(cutoff) || cutoff < prior_cutoff)
                return 2;
            prior_cutoff = cutoff;
#ifdef WIRE
            if (cutoff * hz >= 9007199254740992.)
                return 2;
            struct clock_estimate ce;
            clock_fill(&ce, hz, 0., 0);
            if (stepcompress_flush(sink, clock_from_time(&ce, cutoff)))
                return 4;
#endif
        } else if (op == 'f') {
            double end;
            if (scanf("%lf", &end) != 1)
                return 2;
            for (int q = 0; q < 2; ++q)
                trapq_check_sentinels(queue[q]);
            if (itersolve_generate_steps(sk, sink, end))
                return 3;
            printf("flush %.17g %.17g\n", end, sk->commanded_pos);
        } else {
            return 2;
        }
    }
#ifdef WIRE
    if (stepcompress_flush(sink, UINT64_MAX))
        return 4;
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
        } else {
            return 5;
        }
    }
    stepcompress_free(sink);
    message_queue_free(&messages);
#endif
    extruder_stepper_free(sk);
    trapq_free(queue[0]);
    trapq_free(queue[1]);
    return 0;
}
