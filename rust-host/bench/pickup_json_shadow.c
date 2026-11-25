// SPDX-License-Identifier: GPL-3.0-only
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct rec { uint64_t clock; uint32_t interval; uint16_t count; int16_t add; int kind; };

static char *num(char *dest, uint64_t n) {
    char digits[20]; char *end = digits + sizeof(digits), *cur = end;
    do { *--cur = '0' + n % 10; n /= 10; } while (n);
    memcpy(dest, cur, end - cur);
    return dest + (end-cur);
}

static void compact(const struct rec *r, int comma) {
    char row[128], *p = row;
    if (comma) *p++ = ',';
    if (r->kind) {
        const char *dir = r->kind == 1 ?
            "{\"kind\":\"direction\",\"positive\":false}" :
            "{\"kind\":\"direction\",\"positive\":true}";
        size_t n = strlen(dir);
        memcpy(p, dir, n); p += n;
    } else {
#define LITERAL(s) do { memcpy(p,s,sizeof(s)-1); p += sizeof(s)-1; } while (0)
        LITERAL("{\"kind\":\"steps\",\"interval\":");
        p = num(p, r->interval);
        LITERAL(",\"count\":");
        p = num(p, r->count);
        LITERAL(",\"add\":");
        int add = r->add;
        if (add < 0) { *p++ = '-'; add = -add; }
        p = num(p, (unsigned)add);
        LITERAL(",\"last_clock\":");
        p = num(p, r->clock);
        *p++ = '}';
#undef LITERAL
    }
    fwrite(row, 1, p-row, stdout);
}

static void generic(const struct rec *r, int comma) {
    if (comma) putchar(',');
    if (r->kind) {
        fprintf(stdout, "{\"kind\":\"direction\",\"positive\":%s}",
                r->kind == 1 ? "false" : "true");
    } else {
        fprintf(stdout, "{\"kind\":\"steps\",\"interval\":%" PRIu32
            ",\"count\":%" PRIu16 ",\"add\":%" PRIi16 ",\"last_clock\":%" PRIu64 "}",
            r->interval, r->count, r->add, r->clock);
    }
}

int main(int argc, char **argv) {
    if (argc != 4 || (strcmp(argv[1],"generic") && strcmp(argv[1],"compact"))) return 2;
    FILE *file = fopen(argv[2], "r"); if (!file) return 2;
    struct rec *data = calloc(1048600, sizeof(*data)); if (!data) return 2;
    size_t ends[4], used = 0, lane = 0;
    char line[120];
    for (; fgets(line, sizeof line, file);) {
        if (lane >= 4 || used >= 1048600) return 2;
        if (!strcmp(line, "end\n")) { ends[lane++] = used; continue; }
        struct rec *r = data + used++;
        int dir;
        if (sscanf(line, "d %d", &dir) == 1 && (dir == 0 || dir == 1)) {
            r->kind = dir + 1;
        } else if (sscanf(line, "s %" SCNu32 " %" SCNu16 " %" SCNi16 " %" SCNu64,
            &r->interval, &r->count, &r->add, &r->clock) != 4) return 2;
    }
    if (ferror(file) || fclose(file) || lane != 4) return 2;
    static char buf[8192];
    if (setvbuf(stdout, buf, _IOFBF, sizeof(buf))) return 2;
    struct timespec a,b;
    clock_gettime(CLOCK_MONOTONIC, &a);
    printf("{\"schema\":\"klipper.shared-corexy.runtime-shaper.v1\",\"clock_hz\":%s"
           ",\"lane_order\":[\"corexy_a\",\"corexy_b\",\"z\",\"shared_e\"],\"lanes\":[", argv[3]);
    void (*write_record)(const struct rec *, int) = argv[1][0]=='g' ? generic : compact;
    for (size_t i=0, begin=0; i<4; begin=ends[i++]) {
        fputs(i ? ",[" : "[", stdout);
        for (size_t j=begin; j<ends[i]; ++j) write_record(data+j, j!=begin);
        putchar(']');
    }
    fputs("]}\n", stdout);
    if (fflush(stdout) || ferror(stdout)) return 2;
    clock_gettime(CLOCK_MONOTONIC, &b);
    fprintf(stderr, "%lld\n", (long long)(b.tv_sec-a.tv_sec)*1000000000LL+b.tv_nsec-a.tv_nsec);
    free(data);
    return 0;
}
