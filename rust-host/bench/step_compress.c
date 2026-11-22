// SPDX-License-Identifier: GPL-3.0-only
#include <inttypes.h>
#include "stepcompress.c"

int main(void)
{
    uint64_t base;
    unsigned max_error, count;
    struct qstep *steps = malloc(65535 * sizeof(*steps));
    if (!steps)
        return 2;
    while (scanf("%" SCNu64 " %u %u", &base, &max_error, &count) == 3) {
        if (!count || count > 65535)
            return 2;
        struct stepcompress s = {.last_step_clock = base, .max_error = max_error,
                                 .queue_pos = steps, .queue_next = steps + count};
        for (unsigned i = 0; i < count; i++) {
            uint64_t clock;
            if (scanf("%" SCNu64, &clock) != 1)
                return 2;
            steps[i].clock32 = clock;
        }
        while (s.queue_pos < s.queue_next) {
            struct step_move move = compress_bisect_add(&s);
            if (check_line(&s, move))
                return 3;
            uint64_t n = move.count;
            s.last_step_clock += move.interval * n + (int64_t)move.add * (int64_t)(n * (n - 1) / 2);
            printf("%u %u %d %" PRIu64 "\n", move.interval, move.count, move.add, s.last_step_clock);
            s.queue_pos += move.count;
        }
        puts("end");
    }
    free(steps);
    return 0;
}
