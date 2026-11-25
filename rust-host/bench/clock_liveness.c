/* SPDX-License-Identifier: GPL-3.0-only */
/* Reference policy comparator only. No serial or physical device access. */
#include <stdio.h>
#include <stdbool.h>
int main(void) {
    int fileoutput;
    char actions[513];
    while (scanf("%d %512s", &fileoutput, actions) == 2) {
        unsigned pending = 0, sent = 0, index = 0;
        bool timeout = false;
        for (const char *op = actions; *op; op++) {
            unsigned due = 0;
            switch (*op) {
            case 'q':
                ++sent;
#ifdef SATURATING
                if (pending <= 4) ++pending;
#else
                ++pending;
#endif
                due = index * 1250 + 9839;
                break;
            case 'r': pending = 0; break;
            case 'c': if (pending > 4 && !fileoutput) timeout = true; break;
            default: return 2;
            }
            printf("%d %d %d %u %u\n", pending <= 4, timeout, timeout, sent, due);
            ++index;
        }
    }
    return ferror(stdin) || ferror(stdout);
}
