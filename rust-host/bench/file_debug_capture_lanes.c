// SPDX-License-Identifier: GPL-3.0-only
// Bench only: four independently owned pinned stock step lanes, from captured events.
#define _GNU_SOURCE
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "itersolve.h"
#include "msgblock.h"
#include "stepcompress.h"
#include "trapq.h"

struct stepper_kinematics *corexy_stepper_alloc(char type);
struct stepper_kinematics *cartesian_stepper_alloc(char axis);
int run_e(FILE *in, FILE *out, double hz, unsigned error);
#ifdef CAPTURE_SHAPE
struct stepper_kinematics *input_shaper_alloc(void);
int input_shaper_set_sk(struct stepper_kinematics *, struct stepper_kinematics *);
int input_shaper_set_shaper_params(struct stepper_kinematics *, char, int, double[], double[]);
#endif

enum { LANES = 4, MAX_INPUT = 2 * 1024 * 1024 };
struct input { char *text; size_t size; };
struct lane {
    struct input input;
    char *result;
    size_t size;
    double hz, dist;
    unsigned tolerance;
    int axis, invert, status;
};

static int
load(const char *name, struct input *input)
{
    FILE *f = fopen(name, "rb");
    if (!f)
        return 1;
    int failed = 1;
    if (!fseek(f, 0, SEEK_END)) {
        long size = ftell(f);
        if (size > 0 && size <= MAX_INPUT && !fseek(f, 0, SEEK_SET)) {
            input->text = malloc((size_t)size);
            if (input->text) {
                input->size = (size_t)size;
                failed = fread(input->text, 1, input->size, f) != input->size;
            }
        }
    }
    return fclose(f) || failed;
}

static int
render_xyz(struct lane *lane, FILE *in, FILE *out)
{
    struct list_head messages;
    list_init(&messages);
    struct stepcompress *sink = stepcompress_alloc(&messages);
    struct trapq *queue = trapq_alloc();
    struct stepper_kinematics *sk = lane->axis == 2 ? cartesian_stepper_alloc('z')
        : corexy_stepper_alloc(lane->axis == 0 ? '+' : '-');
    int status = 2;
#ifdef CAPTURE_SHAPE
    struct stepper_kinematics *original = sk;
    sk = input_shaper_alloc();
    if (input_shaper_set_sk(sk, original))
        goto done;
#endif
    stepcompress_fill(sink, 1, lane->tolerance, 3, 4);
    stepcompress_set_invert_sdir(sink, lane->invert);
    stepcompress_set_time(sink, 0., lane->hz);
    itersolve_set_trapq(sk, queue, lane->dist);
    struct clock_estimate ce;
    clock_fill(&ce, lane->hz, 0., 0);
    char op;
    double prior_cutoff = 0.;
    unsigned cutoffs = 0;
    while (fscanf(in, " %c", &op) == 1) {
#ifdef CAPTURE_SHAPE
        if (op == 'h') {
            double a[10], t[10];
            char axis;
            int n;
            if (fscanf(in, " %c %d", &axis, &n) != 2
                || (axis != 'x' && axis != 'y') || n < 0 || n > 10)
                goto done;
            for (int k = 0; k < n; k++)
                if (fscanf(in, "%lf %lf", a+k, t+k) != 2)
                    goto done;
            if (input_shaper_set_shaper_params(sk, axis, n, a, t))
                goto done;
            continue;
        }
#endif
        if (op == 'a') {
            double v[13];
            for (int i = 0; i < 13; ++i)
                if (fscanf(in, "%lf", &v[i]) != 1 || !isfinite(v[i]))
                    goto done;
            trapq_append(queue, v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                         v[7], v[8], v[9], v[10], v[11], v[12]);
        } else if (op == 'b') {
            double x, y, z;
            if (fscanf(in, "%lf %lf %lf", &x, &y, &z) != 3)
                goto done;
            itersolve_set_position(sk, x, y, z);
        } else if (op == 't') {
            double before;
            if (fscanf(in, "%lf", &before) != 1)
                goto done;
            trapq_finalize_moves(queue, before, before);
        } else if (op == 'f') {
            double end;
            if (fscanf(in, "%lf", &end) != 1)
                goto done;
            trapq_check_sentinels(queue);
            if (itersolve_generate_steps(sk, sink, end))
                goto done;
        } else if (op == 'w') {
            double cutoff;
            if (++cutoffs > 512 || fscanf(in, "%lf", &cutoff) != 1
                || !isfinite(cutoff) || cutoff < prior_cutoff
                || cutoff * lane->hz >= 9007199254740992.
                || stepcompress_flush(sink, clock_from_time(&ce, cutoff)))
                goto done;
            prior_cutoff = cutoff;
        } else {
            goto done;
        }
    }
    if (ferror(in) || stepcompress_flush(sink, UINT64_MAX))
        goto done;
    uint64_t tick = 0;
    struct queue_message *msg;
    list_for_each_entry(msg, &messages, node) {
        uint32_t fields[5];
        uint8_t framed[MESSAGE_MAX] = {0};
        memcpy(framed + MESSAGE_HEADER_SIZE, msg->msg, msg->len);
        int len = msg->len + MESSAGE_MIN;
        if (!msgblock_decode(fields, 3, framed, len) && fields[0] == 4) {
            if (fprintf(out, "d %u\n", fields[2]) < 0)
                goto done;
        } else if (!msgblock_decode(fields, 5, framed, len) && fields[0] == 3) {
            int64_t count = fields[3], add = (int32_t)fields[4];
            tick += fields[2] * count + add * count * (count - 1) / 2;
            if (fprintf(out, "s %u %" PRId64 " %" PRId64 " %" PRIu64 "\n",
                        fields[2], count, add, tick) < 0)
                goto done;
        } else {
            goto done;
        }
    }
    status = 0;
done:
    stepcompress_free(sink);
    message_queue_free(&messages);
    trapq_free(queue);
    free(sk);
#ifdef CAPTURE_SHAPE
    free(original);
#endif
    return status;
}

static void *
render(void *arg)
{
    struct lane *lane = arg;
    FILE *in = fmemopen(lane->input.text, lane->input.size, "r");
    FILE *out = open_memstream(&lane->result, &lane->size);
    lane->status = 2;
    if (in && out)
        lane->status = lane->axis == 3 ? run_e(in, out, lane->hz, lane->tolerance)
                                     : render_xyz(lane, in, out);
    int in_error = in ? fclose(in) : 0;
    int out_error = out ? fclose(out) : 0;
    if (in_error || out_error)
        lane->status = 2;
    return NULL;
}

int
main(int argc, char **argv)
{
    if (argc != 12 || (strcmp(argv[1], "1") && strcmp(argv[1], "4")))
        return 2;
    struct input xyz = {0}, e = {0};
    struct lane lanes[LANES] = {0};
    int status = 2;
    if (load(argv[2], &xyz) || load(argv[3], &e))
        goto done;
    double hz = strtod(argv[4], NULL);
    unsigned long tolerance = strtoul(argv[5], NULL, 10);
    if (!isfinite(hz) || hz <= 0. || tolerance > UINT32_MAX)
        goto done;
    for (int i = 0; i < LANES; ++i) {
        lanes[i] = (struct lane){ .input = i == 3 ? e : xyz, .axis = i,
            .hz = hz, .tolerance = (unsigned)tolerance };
        if (i < 3) {
            lanes[i].dist = strtod(argv[6+i], NULL);
            lanes[i].invert = !strcmp(argv[9+i], "1");
            if (!(lanes[i].dist > 0.) || !isfinite(lanes[i].dist)
                || (strcmp(argv[9+i], "0") && strcmp(argv[9+i], "1")))
                goto done;
        }
    }
    if (*argv[1] == '4') {
        pthread_t threads[3];
        int started = 0;
        for (; started < 3; ++started)
            if (pthread_create(&threads[started], NULL, render, &lanes[started]))
                break;
        if (started == 3)
            render(&lanes[3]);
        for (int i = 0; i < started; ++i)
            if (pthread_join(threads[i], NULL))
                abort();
        if (started != 3)
            goto done;
    } else {
        for (int i = 0; i < LANES; ++i)
            render(&lanes[i]);
    }
    for (int i = 0; i < LANES; ++i)
        if (lanes[i].status)
            goto done;
    for (int i = 0; i < LANES; ++i)
        if (fwrite(lanes[i].result, 1, lanes[i].size, stdout) != lanes[i].size
            || fputs("end\n", stdout) == EOF)
            goto done;
    status = fflush(stdout) == EOF;
done:
    for (int i = 0; i < LANES; ++i)
        free(lanes[i].result);
    free(xyz.text);
    free(e.text);
    return status;
}
