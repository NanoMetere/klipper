// SPDX-License-Identifier: GPL-3.0-only
#include <inttypes.h>
#include <time.h>
#include "stepcompress.c"

struct dataset {
    uint64_t base;
    unsigned max_error, count;
    struct qstep *steps;
};

static uint64_t timestamp(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static int run(struct dataset *data, size_t len, int report, uint64_t *checksum)
{
    for (size_t i = 0; i < len; i++) {
        struct dataset *d = &data[i];
        struct stepcompress s = {.last_step_clock = d->base, .max_error = d->max_error,
                                 .queue_pos = d->steps, .queue_next = d->steps + d->count};
        while (s.queue_pos < s.queue_next) {
            struct step_move move = compress_bisect_add(&s);
            if (check_line(&s, move))
                return 3;
            uint64_t n = move.count;
            s.last_step_clock += move.interval * n + (int64_t)move.add * (int64_t)(n * (n - 1) / 2);
            if (report)
                printf("%u %u %d %" PRIu64 "\n", move.interval, move.count, move.add, s.last_step_clock);
            *checksum += s.last_step_clock ^ n ^ (uint16_t)move.add ^ move.interval;
            s.queue_pos += move.count;
        }
        if (report)
            puts("end");
    }
    return 0;
}

int main(int argc, char **argv)
{
    int rounds = argc > 1 ? atoi(argv[1]) : 0;
    struct dataset *data = calloc(4096, sizeof(*data));
    size_t len = 0;
    if (!data)
        return 2;
    for (; len < 4096; len++) {
        struct dataset *d = &data[len];
        if (scanf("%" SCNu64 " %u %u", &d->base, &d->max_error, &d->count) != 3)
            break;
        if (!d->count || d->count > 65535)
            return 2;
        d->steps = malloc(d->count * sizeof(*d->steps));
        if (!d->steps)
            return 2;
        for (unsigned i = 0; i < d->count; i++) {
            uint64_t clock;
            if (scanf("%" SCNu64, &clock) != 1)
                return 2;
            d->steps[i].clock32 = clock;
        }
    }
    uint64_t checksum = 0;
    if (run(data, len, !rounds, &checksum))
        return 3;
    if (rounds) {
        checksum = 0;
        uint64_t start = timestamp();
        for (int i = 0; i < rounds; i++)
            if (run(data, len, 0, &checksum))
                return 3;
        printf("%" PRIu64 " %" PRIu64 "\n", timestamp() - start, checksum);
    }
    for (size_t i=0; i<len; i++)
        free(data[i].steps);
    free(data);
    return 0;
}
