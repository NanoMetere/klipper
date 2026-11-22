// SPDX-License-Identifier: GPL-3.0-only
// Test-only Linux GPIO implementation: no system calls or hardware access.
#include "command.h"
#include "gpio.h"
#include "sched.h"

#define PINS 32

DECL_CONSTANT_STR("GPIO_SIM_ONLY", "rust-host-test-v1");
DECL_ENUMERATION_RANGE("pin", "sim0", 0, PINS);

struct gpio_line {
    uint8_t value;
};
static struct gpio_line lines[PINS];

static struct gpio_line *get_line(uint32_t pin)
{
    if (pin >= PINS)
        shutdown("not a fixture pin");
    return &lines[pin];
}

struct gpio_out gpio_out_setup(uint32_t pin, uint8_t value)
{
    struct gpio_out out = { .line = get_line(pin) };
    out.line->value = !!value;
    return out;
}

void gpio_out_reset(struct gpio_out out, uint8_t value)
{
    out.line->value = !!value;
}

void gpio_out_write(struct gpio_out out, uint8_t value)
{
    out.line->value = !!value;
}

void gpio_out_toggle_noirq(struct gpio_out out)
{
    out.line->value ^= 1;
}

void gpio_out_toggle(struct gpio_out out)
{
    gpio_out_toggle_noirq(out);
}

struct gpio_in gpio_in_setup(uint32_t pin, int8_t pull_up)
{
    struct gpio_in in = { .line = get_line(pin) };
    (void)pull_up;
    return in;
}

void gpio_in_reset(struct gpio_in in, int8_t pull_up)
{
    (void)in;
    (void)pull_up;
}

uint8_t gpio_in_read(struct gpio_in in)
{
    return in.line->value;
}
