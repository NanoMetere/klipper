// SPDX-License-Identifier: GPL-3.0-only
// Benchmark-only counterpart to the bounded canonical Rust scanner.
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static void
scan_line(const uint8_t *s, size_t len, uint8_t *row)
{
    row[0] = 0;
    if (len < 4 || len >= 128 || s[0] != 'G' || s[2] != ' '
        || !(s[1] == '0' || s[1] == '1'))
        return;
    size_t p = 2, n = 1;
    unsigned seen = 0;
    while (p < len) {
        if (s[p] == ';')
            break;
        if (s[p] != ' ')
            return;
        do {
            p++;
        } while (p < len && s[p] == ' ');
        if (p == len || s[p] == ';')
            break;
        uint8_t axis;
        switch (s[p]) {
        case 'X': axis = 1; break;
        case 'Y': axis = 2; break;
        case 'Z': axis = 3; break;
        case 'E': axis = 4; break;
        case 'F': axis = 5; break;
        default: return;
        }
        unsigned bit = 1U << axis;
        if (seen & bit)
            return;
        seen |= bit;
        size_t start = ++p;
        while (p < len) {
            unsigned c = s[p];
            if (!(c - '0' < 10 || c == '+' || c == '-' || c == '.'))
                break;
            p++;
        }
        if (p == start)
            return;
        row[n++] = axis;
        row[n++] = (uint8_t)start;
        row[n++] = (uint8_t)p;
    }
    if (!seen)
        return;
    if (n < 16)
        row[n] = 0;
    row[0] = (uint8_t)(s[1] - '0' + 1);
}

// Same ABI as the Rust library to keep the Python adapter identical.
// Caller provides readable and writable non-overlapping buffers.
int
klipper_rs_move_scan_v1(const uint8_t *src, size_t len, uint8_t *out,
                       size_t out_len)
{
    size_t rows = out_len / 16;
    if (!src || !out || len > 1048576 || !rows || rows > 4096 || out_len % 16)
        return -1;
    size_t count = 0;
    const uint8_t *end = src + len;
    for (;;) {
        if (count == rows)
            return -1;
        const uint8_t *newline = memchr(src, '\n', (size_t)(end - src));
        scan_line(src, (size_t)((newline ? newline : end) - src),
                  out + count++ * 16);
        if (!newline)
            break;
        src = newline + 1;
    }
    return count == rows ? 0 : -1;
}
