// SPDX-License-Identifier: GPL-3.0-only
// Test-only board clock for stock Linux MCU scheduler/stepper logic.
// Time advances only on an explicit fixture command; no wall-time deadlines.
#include "autoconf.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "internal.h"
#include "sched.h"

DECL_CONSTANT("CLOCK_FREQ", CONFIG_CLOCK_FREQ);
DECL_CONSTANT_STR("TIMER_SIM_ONLY", "rust-host-test-v1");

static uint32_t now;
extern uint32_t rust_fixture_next_timer(void);

uint32_t timer_read_time(void)
{
    return now;
}

uint32_t timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}

uint8_t timer_is_before(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}

int timer_check_periodic(uint32_t *ts)
{
    if (timer_is_before(now, *ts))
        return 0;
    *ts = now + timer_from_us(2000000);
    return 1;
}

void timer_kick(void) {}
void timer_disable_signals(void) {}
void timer_enable_signals(void) {}
void irq_disable(void) {}
void irq_enable(void) {}
irqstatus_t irq_save(void) { return 0; }
void irq_restore(irqstatus_t flags) { (void)flags; }
void irq_poll(void) {}
void irq_wait(void) { console_sleep(NULL); }

void command_rust_fixture_advance(uint32_t *args)
{
    uint32_t end = args[0];
    if (timer_is_before(end, now) || (end - now) > timer_from_us(8000000))
        shutdown("fixture clock outside watchdog-free test range");
    for (uint32_t n = 0; n < 2000000; n++) {
        uint32_t next = rust_fixture_next_timer();
        if (timer_is_before(end, next)) {
            now = end;
            return;
        }
        if (timer_is_before(now, next))
            now = next;
        sched_timer_dispatch();
    }
    shutdown("fixture timer callback budget exhausted");
}
DECL_COMMAND(command_rust_fixture_advance, "rust_fixture_advance clock=%u");
