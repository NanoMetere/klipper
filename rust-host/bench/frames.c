// SPDX-License-Identifier: GPL-3.0-only
#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "msgblock.h"

int main(int argc, char **argv) {
    if (argc != 2)
        return 1;
    FILE *file = fopen(argv[1], "rb");
    uint8_t frames[4][64];
    if (!file || fread(frames, 1, sizeof(frames), file) != sizeof(frames))
        return 1;
    fclose(file);
    uint8_t need_sync = 0;
    uint64_t sum = 0;
    const unsigned iterations = 2000000;
    struct timespec begin, end;
    clock_gettime(CLOCK_MONOTONIC, &begin);
    for (unsigned i = 0; i < iterations; i++) {
        uint8_t *frame = frames[i & 3];
        int checked = msgblock_check(&need_sync, frame, frame[0]);
        if (checked <= 0)
            return 1;
        sum += checked;
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    if (need_sync || sum != iterations / 4 * (5 + 8 + 11 + 64))
        return 1;
    double ns = (end.tv_sec - begin.tv_sec) * 1e9 + end.tv_nsec - begin.tv_nsec;
    printf("%.3f\n", ns / iterations);
    return 0;
}
