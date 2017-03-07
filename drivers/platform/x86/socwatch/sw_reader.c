#include "sw_internal.h"
#include "sw_output_buffer.h"
#include "sw_defines.h"

#define SW_BUFFER_CLEANUP_TIMER_DELAY_NSEC 1000000 /* delay buffer cleanup by 10^6 nsec i.e. 1 msec */

/*
 * The alarm queue.
 */
wait_queue_head_t sw_reader_queue;
/*
 * Reader wakeup timer.
 */
static struct hrtimer s_reader_wakeup_timer;
/*
 * Variable to track # timer fires.
 */
static int s_num_timer_fires = 0;

/*
 * The alarm callback.
 */
static enum hrtimer_restart sw_wakeup_callback_i(struct hrtimer *timer)
{
    ++s_num_timer_fires;
    wake_up_interruptible(&sw_reader_queue);
    return HRTIMER_NORESTART;
}

/*
 * Init reader queue.
 */
int sw_init_reader_queue(void)
{
    init_waitqueue_head(&sw_reader_queue);
    /*
     * Also init wakeup timer (used in low-overhead mode).
     */
    hrtimer_init(&s_reader_wakeup_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    s_reader_wakeup_timer.function = &sw_wakeup_callback_i;

    return PW_SUCCESS;
}
/*
 * Destroy reader queue.
 */
void sw_destroy_reader_queue(void)
{
    /* NOP */
}
/*
 * Wakeup client waiting for a full buffer.
 */
void sw_wakeup_reader(enum sw_wakeup_action action)
{
    if (!waitqueue_active(&sw_reader_queue)) {
        return;
    }
    /*
     * Direct mode?
     */
    switch (action) {
        case SW_WAKEUP_ACTION_DIRECT:
            wake_up_interruptible(&sw_reader_queue);
            break;
        case SW_WAKEUP_ACTION_TIMER:
            if (!hrtimer_active(&s_reader_wakeup_timer)) {
                ktime_t ktime = ns_to_ktime(SW_BUFFER_CLEANUP_TIMER_DELAY_NSEC);
                // TODO: possible race here -- introduce locks?
                hrtimer_start(&s_reader_wakeup_timer, ktime, HRTIMER_MODE_REL);
            }
            break;
        default:
            break;
    }
    return;
}
/*
 * Wakeup client waiting for a full buffer, and
 * cancel any timers initialized by the reader
 * subsys.
 */
void sw_cancel_reader(void)
{
    /*
     * Cancel pending wakeup timer (used in low-overhead mode).
     */
    if (hrtimer_active(&s_reader_wakeup_timer)) {
        hrtimer_cancel(&s_reader_wakeup_timer);
    }
    /*
     * There might be a reader thread blocked on a read: wake
     * it up to give it a chance to respond to changed
     * conditions.
     */
    sw_wakeup_reader(SW_WAKEUP_ACTION_DIRECT);
}

void sw_print_reader_stats(void)
{
#if DO_OVERHEAD_MEASUREMENTS
    printk(KERN_INFO "# reader queue timer fires = %d\n", s_num_timer_fires);
#endif // OVERHEAD
}
