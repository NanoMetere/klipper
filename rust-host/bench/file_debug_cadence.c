// SPDX-License-Identifier: GPL-3.0-only
#include <math.h>
#include <stdio.h>
int main(void)
{
    double need, gen, wire;
    while (scanf("%lf %lf %lf", &need, &gen, &wire) == 3) {
        if (!isfinite(need) || !isfinite(gen) || !isfinite(wire)
            || need < 0. || gen < 0. || wire < 0.
            || need > 120.25 || gen > 120.25 || wire > gen)
            return 2;
        double faux = need - 1.5, delta = 0.700 - 0.450;
        unsigned count = 0;
        while (gen < faux) {
            double target = gen + delta;
            if (count > 100 && faux > target)
                target += trunc((faux-target)/delta) * delta;
            wire = fmax(wire, target - 0.050);
            gen = fmax(fmax(gen, target), wire);
            printf("%.17e %.17e\n", wire, gen);
            count++;
        }
        if (!count) {
            wire = fmax(wire, need+0.250);
            gen = fmax(gen, wire);
            printf("%.17e %.17e\n", wire, gen);
        }
        printf("end %d\n", count != 0);
    }
}
