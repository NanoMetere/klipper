// SPDX-License-Identifier: GPL-3.0-only
// Bench-only deterministic lane scheduling over one captured stock motion plan.
#define _GNU_SOURCE
#include <pthread.h>
#include <stdatomic.h>
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


struct shape { int n; double a[10], t[10]; };
enum { MAX_TOOLS = 3, MAX_LANES = 6, MAX_WORKERS = 4 };
struct tool_queue {
    struct trapq *queue;
    int n_params;
    double params[8][3];
};
struct input {
    struct trapq *q;
    struct tool_queue tools[MAX_TOOLS];
    int shaped;
    struct shape shapes[3];
};
struct lane {
    double xpos, ypos, zpos, step, last, hz;
    unsigned err, invert, tool;
    char axis;
    char *bytes;
    size_t len;
    int status;
};


static int solve_lane(struct input *ctx, struct lane *lane)
{
    double xpos=lane->xpos, ypos=lane->ypos, zpos=lane->zpos;
    double step=lane->step, last=lane->last, hz=lane->hz;
    unsigned err=lane->err, invert=lane->invert;
    char axis=lane->axis;
    struct shape *shapes=ctx->shapes;
    struct tool_queue *tool = &ctx->tools[lane->tool];
    double (*params)[3] = tool->params;
    struct trapq *q = ctx->q, *eq = tool->queue;
    int n_params = tool->n_params, shaped = ctx->shaped;
    FILE *out = open_memstream(&lane->bytes, &lane->len);
    if (!out)
        return 9;
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
            fprintf(out, "d %u\n", fields[2]);
        } else if (!msgblock_decode(fields, 5, framed, len) && fields[0] == 3) {
            int64_t count = fields[3], add = (int32_t)fields[4];
            tick += fields[2] * count + add * count * (count - 1) / 2;
            fprintf(out, "s %u %" PRId64 " %" PRId64 " %" PRIu64 "\n",
                   fields[2], count, add, tick);
        } else
            return 4;
    }
    fputs("end\n",out);
    if (axis == 'e') extruder_stepper_free(sk);
    else {
        if (original != sk)
            free(original);
        free(sk);
    }
    stepcompress_free(sc);
    message_queue_free(&messages);

    return fclose(out) ? 9 : 0;
}
struct worker {
    int id, count, workers;
    struct input *ctx;
    struct lane *lanes;
    atomic_int *next;
};
static void *worker_main(void *pointer)
{
    struct worker *w = pointer;
    // Match Rust's static and dynamic scheduling choices; result order
    // remains the lane-request order, regardless of completion order.
    if (w->workers <= 2 || w->workers >= w->count) {
        for (int i = w->id; i < w->count; i += w->workers)
            w->lanes[i].status = solve_lane(w->ctx, &w->lanes[i]);
    } else {
        for (;;) {
            int i = atomic_fetch_add_explicit(w->next, 1, memory_order_relaxed);
            if (i >= w->count)
                break;
            w->lanes[i].status = solve_lane(w->ctx, &w->lanes[i]);
        }
    }
    return NULL;
}
int main(int argc, char **argv)
{
    if (argc!=2 || strlen(argv[1])!=1 || !strchr("1234", argv[1][0]))
        return 2;
    int workers=argv[1][0]-'0', count=0;
    struct input ctx = {.q = trapq_alloc()};
    for (int i = 0; i < MAX_TOOLS; i++)
        ctx.tools[i].queue = trapq_alloc();
    unsigned current = 0;
    struct trapq *q = ctx.q;
    struct lane lanes[MAX_LANES] = {0};
    struct shape *shapes = ctx.shapes;
    char op;
    uint64_t processing = 0;
    while (scanf(" %c", &op) == 1) {
        if (count && op != 's' && op != 'u')
            return 2;
        if (op == 't') {
            if (scanf("%u", &current) != 1 || current >= MAX_TOOLS)
                return 2;
        } else if (op == 'm' || op == 'e') {
            double at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &at, &a, &c, &d, &x, &y, &z, &rx, &ry, &rz, &sv, &cv, &accel) != 13)
                return 2;
            uint64_t started = machine_trace_cpu_ns();
            trapq_append(op == 'm' ? q : ctx.tools[current].queue, at, a, c, d, x, y, z, rx, ry, rz, sv, cv, accel);
            processing += machine_trace_cpu_ns() - started;
        } else if (op == 'g') {
            struct tool_queue *tool = &ctx.tools[current];
            if (tool->n_params >= 8)
                return 2;
            double *row = tool->params[tool->n_params];
            if (scanf("%lf %lf %lf", &row[0], &row[1], &row[2]) != 3)
                return 2;
            tool->n_params++;
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
            ctx.shaped = 1;
        } else if (op == 's' || op == 'u') {
            if (count==MAX_LANES)
                return 2;
            struct lane *l = &lanes[count++];
            if (op == 'u' && (scanf("%u", &l->tool) != 1 || l->tool >= MAX_TOOLS))
                return 2;
            if (scanf(" %c %lf %lf %lf %lf %lf %lf %u %u",
                &l->axis,&l->xpos,&l->ypos,&l->zpos,&l->step,&l->last,&l->hz,&l->err,&l->invert)!=9
                || !strchr("xyz+-e", l->axis) || (op == 'u' && l->axis != 'e'))
                return 2;
        } else return 2;
    }
    if (!count) return 2;
    // Finalize sentinels before any solver can read either shared queue.
    trapq_check_sentinels(q);
    for (int i = 0; i < MAX_TOOLS; i++)
        trapq_check_sentinels(ctx.tools[i].queue);
    if (workers>count) workers=count;
    struct worker runner[MAX_WORKERS];
    pthread_t threads[MAX_WORKERS];
    atomic_int next = 0;
    int spawned=0, failed=0;
    for (int i=0; i<workers; ++i)
        runner[i]=(struct worker){i,count,workers,&ctx,lanes,&next};
    for (int i=1; i<workers; ++i) {
        if (pthread_create(&threads[i],NULL,worker_main,&runner[i])) {
            failed=1; break;
        }
        spawned=i;
    }
    if (!failed) worker_main(&runner[0]);
    for (int i=1; i<=spawned; ++i)
        if (pthread_join(threads[i],NULL)) abort();
    for (int i=0; i<count; ++i)
        if (lanes[i].status) failed=1;
    for (int i=0; i<count; ++i) {
        if (!failed && fwrite(lanes[i].bytes,1,lanes[i].len,stdout)!=lanes[i].len)
            failed=1;
        free(lanes[i].bytes);
    }
    if (failed) return 3;
    trapq_free(q);
    for (int i = 0; i < MAX_TOOLS; i++)
        trapq_free(ctx.tools[i].queue);
    fprintf(stderr, "TIMED_NS %" PRIu64 "\n", processing);
    return 0;
}
