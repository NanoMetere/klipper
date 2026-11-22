// SPDX-License-Identifier: GPL-3.0-only
#include <stdio.h>
#include <stdlib.h>
#include "itersolve.c"

struct stepcompress { int dir; };
int stepcompress_get_step_dir(struct stepcompress *s) { return s->dir; }
int stepcompress_commit(struct stepcompress *s) { puts("commit"); return 0; }
int stepcompress_append(struct stepcompress *s, int dir, double start, double offset)
{
    s->dir = dir;
    printf("step %d %.17g %.17g\n", dir, start, offset);
    return 0;
}
struct stepper_kinematics *cartesian_stepper_alloc(char axis);
struct stepper_kinematics *corexy_stepper_alloc(char axis);

int main(void)
{
    struct stepcompress sink = {0};
    struct stepper_kinematics *sk = NULL;
    char op;
    while (scanf(" %c", &op) == 1) {
        if (op == 'r') {
            char axis;
            double dist, x, y, z;
            if (scanf(" %c %lf %lf %lf %lf", &axis, &dist, &x, &y, &z) != 5)
                return 2;
            free(sk);
            sk = axis == '+' || axis == '-' ? corexy_stepper_alloc(axis) : cartesian_stepper_alloc(axis);
            sk->step_dist = dist;
            itersolve_set_position(sk, x, y, z);
            sink.dir = 0;
        } else if (op == 'm') {
            struct move m = {0};
            double start, end;
            if (scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &m.print_time, &m.move_t, &m.start_pos.x, &m.start_pos.y, &m.start_pos.z,
                      &m.axes_r.x, &m.axes_r.y, &m.axes_r.z, &m.start_v, &m.half_accel,
                      &start, &end) != 12 || !sk)
                return 2;
            if (check_active(sk, &m) && itersolve_gen_steps_range(sk, &sink, &m, start, end))
                return 3;
            printf("pos %.17g\n", sk->commanded_pos);
        } else
            return 2;
    }
    free(sk);
    return 0;
}
