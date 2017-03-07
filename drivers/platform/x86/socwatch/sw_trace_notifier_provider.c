#include <linux/version.h> // "LINUX_VERSION_CODE"
#include <linux/hrtimer.h>
#include <asm/cputime.h>
#include <asm/hardirq.h>
#include <asm/local.h>

#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
    #include <asm/trace/irq_vectors.h> // for the various APIC vector tracepoints (e.g. "thermal_apic", "local_timer" etc.)
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
struct pool_workqueue; // Forward declaration to avoid compiler warnings
struct cpu_workqueue_struct; // Forward declaration to avoid compiler warnings
#include <trace/events/workqueue.h>
#include <linux/suspend.h> // for 'pm_notifier'

#include "sw_collector.h"
#include "sw_overhead_measurements.h"
#include "sw_tracepoint_handlers.h"
#include "sw_output_buffer.h"
#include "sw_trace_notifier_provider.h"

/* -------------------------------------------------
 * Compile time constants and useful macros.
 * -------------------------------------------------
 */
#ifndef __get_cpu_var
    /*
     * Kernels >= 3.19 don't include a definition
     * of '__get_cpu_var'. Create one now.
     */
    #define __get_cpu_var(var) *this_cpu_ptr(&var)
#endif // __get_cpu_var

#define BEGIN_LOCAL_IRQ_STATS_READ(p) do{	\
    p = &__get_cpu_var(irq_stat);

#define END_LOCAL_IRQ_STATS_READ(p)		\
    }while(0)
/*
 * CAS{32,64}
 */
#define CAS32(p, o, n) ( cmpxchg((p), (o), (n)) == (o) )
#define CAS64(p, o, n) ( cmpxchg64((p), (o), (n)) == (o) )
/*
 * Timer start pid accessor macros
 */
#ifdef CONFIG_TIMER_STATS
    #define GET_TIMER_THREAD_ID(t) ( (t)->start_pid ) /* 'start_pid' is actually the thread ID of the thread that initialized the timer */
#else
    #define GET_TIMER_THREAD_ID(t) ( -1 )
#endif // CONFIG_TIMER_STATS
/*
 * Tracepoint probe register/unregister functions and
 * helper macros.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    #define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe) WARN_ON(register_trace_##name(probe))
    #define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe) unregister_trace_##name(probe)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0)
    #define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe) WARN_ON(register_trace_##name(probe, NULL))
    #define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe) unregister_trace_##name(probe, NULL)
#else
    #define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe) WARN_ON(tracepoint_probe_register(node->tp, probe, NULL))
    #define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe) tracepoint_probe_unregister(node->tp, probe, NULL)
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    #define _DEFINE_PROBE_FUNCTION(name, ...) static void name(__VA_ARGS__)
#else
    #define _DEFINE_PROBE_FUNCTION(name, ...) static void name(void *ignore, __VA_ARGS__)
#endif
#define DEFINE_PROBE_FUNCTION(x) _DEFINE_PROBE_FUNCTION(x)

/*
 * Tracepoint probe function parameters.
 * These tracepoint signatures depend on kernel version.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
    #define PROBE_TPS_PARAMS sw_probe_power_start_i, unsigned int type, unsigned int state
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    #define PROBE_TPS_PARAMS sw_probe_power_start_i, unsigned int type, unsigned int state, unsigned int cpu_id
#else
    #define PROBE_TPS_PARAMS sw_probe_cpu_idle_i, unsigned int state, unsigned int cpu_id
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    #define PROBE_TPF_PARAMS sw_probe_power_frequency_i, unsigned int type, unsigned int state
#else
    #define PROBE_TPF_PARAMS sw_probe_cpu_frequency_i, unsigned int new_freq, unsigned int cpu
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    #define PROBE_SCHED_WAKEUP_PARAMS sw_probe_sched_wakeup_i, struct rq *rq, struct task_struct *task, int success
#else
    #define PROBE_SCHED_WAKEUP_PARAMS sw_probe_sched_wakeup_i, struct task_struct *task, int success
#endif

#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        #define PROBE_WAKE_LOCK_PARAMS sw_probe_wake_lock_i, struct wake_lock *lock
        #define PROBE_WAKE_UNLOCK_PARAMS sw_probe_wake_unlock_i, struct wake_unlock *unlock
    #else
        #define PROBE_WAKE_LOCK_PARAMS sw_probe_wakeup_source_activate_i, const char *name, unsigned int state
        #define PROBE_WAKE_UNLOCK_PARAMS sw_probe_wakeup_source_deactivate_i, const char *name, unsigned int state
    #endif // version
#endif // CONFIG_SOCWATCH_ANDROID

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
    #define PROBE_WORKQUEUE_PARAMS sw_probe_workqueue_execution_i, struct task_struct *wq_thread, struct work_struct *work
#else
    #define PROBE_WORKQUEUE_PARAMS sw_probe_workqueue_execute_start_i, struct work_struct *work
#endif

#define PROBE_SCHED_SWITCH_PARAMS sw_probe_sched_switch_i, struct task_struct *prev, struct task_struct *next
/*
 * These tracepoint signatures are independent of kernel version.
 */
#define PROBE_IRQ_PARAMS sw_probe_irq_handler_entry_i, int irq, struct irqaction *action
#define PROBE_TIMER_ARGS sw_probe_timer_expire_entry_i, struct timer_list *t
#define PROBE_HRTIMER_PARAMS sw_probe_hrtimer_expire_entry_i, struct hrtimer *hrt, ktime_t *now
#define PROBE_PROCESS_FORK_PARAMS sw_probe_sched_process_fork_i, struct task_struct *parent, struct task_struct *child
#define PROBE_SCHED_PROCESS_EXIT_PARAMS sw_probe_sched_process_exit_i, struct task_struct *task
#define PROBE_THERMAL_APIC_ENTRY_PARAMS sw_probe_thermal_apic_entry_i, int vector
#define PROBE_THERMAL_APIC_EXIT_PARAMS sw_probe_thermal_apic_exit_i, int vector

#define IS_VALID_WAKEUP_EVENT(cpu) ({ \
        bool *per_cpu_event = &per_cpu(sw_is_valid_wakeup_event, (cpu)); \
        bool old_value = CAS32(per_cpu_event, true, sw_wakeup_event_flag); \
        old_value; \
        })
#define SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu) (IS_VALID_WAKEUP_EVENT(cpu))
#define RESET_VALID_WAKEUP_EVENT_COUNTER(cpu) (per_cpu(sw_is_valid_wakeup_event, (cpu)) = true)

#define NUM_TRACEPOINT_NODES SW_ARRAY_SIZE(s_trace_collector_lists)
#define NUM_VALID_TRACEPOINTS (NUM_TRACEPOINT_NODES - 1) /* "-1" for IPI */
#define FOR_EACH_TRACEPOINT_NODE(idx, node) for (idx=0; idx<NUM_TRACEPOINT_NODES && (node=&s_trace_collector_lists[idx]); ++idx)

#define FOR_EACH_NOTIFIER_NODE(idx, node) for (idx=0; idx<SW_ARRAY_SIZE(s_notifier_collector_lists) && (node = &s_notifier_collector_lists[idx]); ++idx)
/*
 * Use these macros if all tracepoint ID numbers ARE contiguous from 0 -- max tracepoint ID #
 */
#if 0
#define IS_VALID_TRACE_NOTIFIER_ID(id) ( (id) >= 0 && (id) < SW_ARRAY_SIZE(s_trace_collector_lists) )
#define GET_COLLECTOR_TRACE_NODE(id) (&s_trace_collector_lists[id])
#define FOR_EACH_trace_notifier_id(idx) for (idx=0; idx < SW_ARRAY_SIZE(s_trace_collector_lists); ++idx)
#endif // if 0
/*
 * Use these macros if all tracepoint ID numbers are NOT contiguous from 0 -- max tracepoint ID #
 */
#define GET_COLLECTOR_TRACE_NODE(idx) ({int __idx=0; struct sw_trace_notifier_data *__node=NULL, *__retVal=NULL; \
        FOR_EACH_TRACEPOINT_NODE(__idx, __node) { \
            if ((idx) == GET_TRACE_NOTIFIER_ID(__node)) { \
                __retVal = __node; break; \
            } \
        } \
        __retVal;})
#define IS_VALID_TRACE_NOTIFIER_ID(idx) (GET_COLLECTOR_TRACE_NODE(idx) != NULL)

#define GET_COLLECTOR_NOTIFIER_NODE(idx) ({int __idx=0; struct sw_trace_notifier_data *__node=NULL, *__retVal=NULL; \
        FOR_EACH_NOTIFIER_NODE(__idx, __node) { \
            if ((idx) == GET_TRACE_NOTIFIER_ID(__node)) { \
                __retVal = __node; break; \
            } \
        } \
        __retVal;})
#define IS_VALID_NOTIFIER_ID(idx) (GET_COLLECTOR_NOTIFIER_NODE(idx) != NULL)

/* -------------------------------------------------
 * Local function declarations.
 * -------------------------------------------------
 */
/*
 * The tracepoint registration functions.
 */
int sw_register_trace_cpu_idle_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_cpu_idle_i(struct sw_trace_notifier_data *node);
int sw_register_trace_cpu_frequency_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_cpu_frequency_i(struct sw_trace_notifier_data *node);
int sw_register_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_wakeup_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_wakeup_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_process_fork_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_process_fork_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_process_exit_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_process_exit_i(struct sw_trace_notifier_data *node);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
    int sw_register_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node);
    int sw_unregister_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node);
    int sw_register_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node);
    int sw_unregister_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node);
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        int sw_register_trace_wake_lock_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wake_lock_i(struct sw_trace_notifier_data *node);
        int sw_register_trace_wake_unlock_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wake_unlock_i(struct sw_trace_notifier_data *node);
    #else // LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
        int sw_register_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node);
        int sw_register_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node);
    #endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#endif // CONFIG_SOCWATCH_ANDROID
int sw_register_trace_workqueue_execution_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_workqueue_execution_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_switch_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_switch_i(struct sw_trace_notifier_data *node);
int sw_register_suspend_notifier_i(struct sw_trace_notifier_data *node);
int sw_unregister_suspend_notifier_i(struct sw_trace_notifier_data *node);
void sw_handle_sched_wakeup_i(struct sw_collector_data *node, int source_cpu, int target_cpu);
void sw_handle_timer_wakeup_helper_i(struct sw_collector_data *curr, struct sw_trace_notifier_data *node,
				     pid_t tid);
void sw_handle_apic_timer_wakeup_i(struct sw_collector_data *node);
void sw_handle_workqueue_wakeup_helper_i(int cpu, struct sw_collector_data *node);
void sw_handle_sched_switch_helper_i(void);
void sw_tps_apic_i(int cpu);
void sw_tps_tps_i(int cpu);
void sw_tps_wakeup_i(int cpu);
void sw_tps_i(void);
void sw_tpf_i(int cpu, struct sw_trace_notifier_data *node);
void sw_process_fork_exit_helper_i(struct sw_collector_data *node, struct task_struct *task, bool is_fork);
void sw_produce_wakelock_msg_i(int cpu, struct sw_collector_data *node, const char *name,
			       int type, u64 timeout, int pid, int tid, const char *proc_name);
u64 sw_my_local_arch_irq_stats_cpu_i(void);

/*
 * The tracepoint probes.
 */
/*
 * The tracepoint handlers.
 */
void sw_handle_trace_notifier_i(struct sw_trace_notifier_data *node);
void sw_handle_trace_notifier_on_cpu_i(int cpu, struct sw_trace_notifier_data *node);

/* -------------------------------------------------
 * Variable definitions.
 * -------------------------------------------------
 */
/*
 * For overhead measurements.
 */
DECLARE_OVERHEAD_VARS(sw_handle_timer_wakeup_helper_i); // for the "timer_expire" family of probes
DECLARE_OVERHEAD_VARS(sw_handle_irq_wakeup_i); // for IRQ wakeups
DECLARE_OVERHEAD_VARS(sw_handle_sched_wakeup_i); // for SCHED
DECLARE_OVERHEAD_VARS(sw_tps_i); // for TPS
DECLARE_OVERHEAD_VARS(sw_tpf_i); // for TPF
DECLARE_OVERHEAD_VARS(sw_process_fork_exit_helper_i);
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    DECLARE_OVERHEAD_VARS(sw_handle_wakelock_i); // for wake lock/unlock
#endif // CONFIG_SOCWATCH_ANDROID
DECLARE_OVERHEAD_VARS(sw_handle_workqueue_wakeup_helper_i);
DECLARE_OVERHEAD_VARS(sw_handle_sched_switch_helper_i);
/*
 * Per-cpu wakeup counters.
 * Used to decide which wakeup event is the first to occur after a
 * core wakes up from a C-state.
 * Set to 'true' in TPS probe
 */
static DEFINE_PER_CPU(bool, sw_is_valid_wakeup_event) = {true};
/*
 * Per-cpu counts of the number of times the local APIC fired.
 * We need a separate count because some apic timer fires don't seem
 * to result in hrtimer/timer expires
 */
static DEFINE_PER_CPU(u64, sw_num_local_apic_timer_inters) = 0;
/*
 * Flag value to use to decide if the event is a valid wakeup event.
 * Set to 'false' in TPS probe.
 */
static bool sw_wakeup_event_flag = true;
/*
 * Scheduler-based polling emulation.
 */
DEFINE_PER_CPU(unsigned long, sw_pcpu_polling_jiff);
pw_u16_t sw_min_polling_interval_msecs;

/*
 * IDs for supported tracepoints.
 */
enum sw_trace_id {
    SW_TRACE_ID_CPU_IDLE = 0,
    SW_TRACE_ID_CPU_FREQUENCY,
    SW_TRACE_ID_IRQ_HANDLER_ENTRY,
    SW_TRACE_ID_TIMER_EXPIRE_ENTRY,
    SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY,
    SW_TRACE_ID_SCHED_WAKEUP,
    SW_TRACE_ID_IPI,
    SW_TRACE_ID_SCHED_PROCESS_FORK,
    SW_TRACE_ID_SCHED_PROCESS_EXIT,
    SW_TRACE_ID_THERMAL_APIC_ENTRY,
    SW_TRACE_ID_THERMAL_APIC_EXIT,
    SW_TRACE_ID_WAKE_LOCK,
    SW_TRACE_ID_WAKE_UNLOCK,
    SW_TRACE_ID_WORKQUEUE_EXECUTE_START,
    SW_TRACE_ID_SCHED_SWITCH,
};
/*
 * IDs for supported notifiers.
 */
enum sw_notifier_id {
    SW_NOTIFIER_ID_SUSPEND_NOTIFIER=1,
};

/*
 * Names for supported tracepoints. A tracepoint
 * 'name' consists of two strings: a "kernel" string
 * that is used to locate the tracepoint within the kernel
 * and an "abstract" string, that is used by Ring-3 to
 * specify which tracepoints to use during a collection.
 */
static const struct sw_trace_notifier_name s_trace_names[] = {
    [SW_TRACE_ID_CPU_IDLE] = {"cpu_idle", "CPU-IDLE"},
    [SW_TRACE_ID_CPU_FREQUENCY] = {"cpu_frequency", "CPU-FREQUENCY"},
    [SW_TRACE_ID_IRQ_HANDLER_ENTRY] = {"irq_handler_entry", "IRQ-ENTRY"},
    [SW_TRACE_ID_TIMER_EXPIRE_ENTRY] = {"timer_expire_entry", "TIMER-ENTRY"},
    [SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY] = {"hrtimer_expire_entry", "HRTIMER-ENTRY"},
    [SW_TRACE_ID_SCHED_WAKEUP] = {"sched_wakeup", "SCHED-WAKEUP"},
    [SW_TRACE_ID_IPI] = {NULL, "IPI"},
    [SW_TRACE_ID_SCHED_PROCESS_FORK] = {"sched_process_fork", "PROCESS-FORK"},
    [SW_TRACE_ID_SCHED_PROCESS_EXIT] = {"sched_process_exit", "PROCESS-EXIT"},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
        [SW_TRACE_ID_THERMAL_APIC_ENTRY] = {"thermal_apic_entry", "THERMAL-THROTTLE-ENTRY"},
        [SW_TRACE_ID_THERMAL_APIC_EXIT] = {"thermal_apic_exit", "THERMAL-THROTTLE-EXIT"},
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
            [SW_TRACE_ID_WAKE_LOCK] = {"wake_lock", "WAKE-LOCK"},
            [SW_TRACE_ID_WAKE_UNLOCK] = {"wake_unlock", "WAKE-UNLOCK"},
    #else // LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
            [SW_TRACE_ID_WAKE_LOCK] = {"wakeup_source_activate", "WAKE-LOCK"},
            [SW_TRACE_ID_WAKE_UNLOCK] = {"wakeup_source_deactivate", "WAKE-UNLOCK"},
    #endif
#endif
    [SW_TRACE_ID_WORKQUEUE_EXECUTE_START] = {"workqueue_execute_start", "WORKQUEUE-START"},
    [SW_TRACE_ID_SCHED_SWITCH] = {"sched_switch", "CONTEXT-SWITCH"},
};

/*
 * Names for supported notifiers. A notifier
 * 'name' consists of two strings: an unused "kernel" string
 * and an "abstract" string, that is used by Ring-3 to
 * specify which notifiers to use during a collection.
 */
static const struct sw_trace_notifier_name s_notifier_names[] = {
    [SW_NOTIFIER_ID_SUSPEND_NOTIFIER] = {"suspend_notifier" /* don't care */, "SUSPEND-NOTIFIER"},
};

/*
 * Macros to retrieve tracepoint and notifier IDs.
 */
#define GET_TRACE_ID_FROM_NODE(node) ( (node)->name - s_trace_names )
#define GET_NOTIFIER_ID_FROM_NODE(node) ( (node)->name - s_notifier_names )

#define GET_TRACE_NOTIFIER_ID(node) (int)( ( (node)->type == SW_TRACE_COLLECTOR_TRACEPOINT) ? GET_TRACE_ID_FROM_NODE(node) : GET_NOTIFIER_ID_FROM_NODE(node) )

/*
 * A list of supported tracepoints.
 */
static struct sw_trace_notifier_data s_trace_collector_lists[] = {
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_CPU_IDLE], &sw_register_trace_cpu_idle_i, &sw_unregister_trace_cpu_idle_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_CPU_FREQUENCY], &sw_register_trace_cpu_frequency_i, &sw_unregister_trace_cpu_frequency_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_IRQ_HANDLER_ENTRY], &sw_register_trace_irq_handler_entry_i, &sw_unregister_trace_irq_handler_entry_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_TIMER_EXPIRE_ENTRY], &sw_register_trace_timer_expire_entry_i, &sw_unregister_trace_timer_expire_entry_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY], &sw_register_trace_hrtimer_expire_entry_i, &sw_unregister_trace_hrtimer_expire_entry_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_SCHED_WAKEUP], &sw_register_trace_sched_wakeup_i, &sw_unregister_trace_sched_wakeup_i, NULL},
    /* Placeholder for IPI -- no tracepoints associated with it! */
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_IPI], NULL, NULL, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_SCHED_PROCESS_FORK], &sw_register_trace_sched_process_fork_i, &sw_unregister_trace_sched_process_fork_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_SCHED_PROCESS_EXIT], &sw_register_trace_sched_process_exit_i, &sw_unregister_trace_sched_process_exit_i, NULL},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
        /*
         * For thermal throttling.
         * We probably only need one of either 'entry' or 'exit'. Use
         * both, until we decide which one to keep. Note that
         * tracepoint IDs for these, and subsequent tracepoints
         * (e.g. 'wake_lock') will change once we've picked which
         * one to use.
         */
        {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_THERMAL_APIC_ENTRY], &sw_register_trace_thermal_apic_entry_i, &sw_unregister_trace_thermal_apic_entry_i, NULL},
        {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_THERMAL_APIC_EXIT], &sw_register_trace_thermal_apic_exit_i, &sw_unregister_trace_thermal_apic_exit_i, NULL},
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
    /* Wakelocks have multiple tracepoints, depending on kernel version */
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
            {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_LOCK], &sw_register_trace_wake_lock_i, &sw_unregister_trace_wake_lock_i, NULL},
            {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_UNLOCK], &sw_register_trace_wake_unlock_i, &sw_unregister_trace_wake_unlock_i, NULL},
    #else // LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
            {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_LOCK], &sw_register_trace_wakeup_source_activate_i, &sw_unregister_trace_wakeup_source_activate_i, NULL},
            {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_UNLOCK], &sw_register_trace_wakeup_source_deactivate_i, &sw_unregister_trace_wakeup_source_deactivate_i, NULL},
    #endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#endif // CONFIG_SOCWATCH_ANDROID
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WORKQUEUE_EXECUTE_START], &sw_register_trace_workqueue_execution_i, &sw_unregister_trace_workqueue_execution_i, NULL},
    {SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_SCHED_SWITCH], &sw_register_trace_sched_switch_i, &sw_unregister_trace_sched_switch_i, NULL},
};
/*
 * List of supported notifiers.
 */
static struct sw_trace_notifier_data s_notifier_collector_lists[] = {
    {SW_TRACE_COLLECTOR_NOTIFIER, &s_notifier_names[SW_NOTIFIER_ID_SUSPEND_NOTIFIER], &sw_register_suspend_notifier_i, &sw_unregister_suspend_notifier_i},
};


/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */
/*
 * Retrieve a TSC value
 * TODO: move to a separate file (e.g. "sw_utils.c")
 */
static inline u64 tscval(void)
{
    u64 tsc = 0;
    rdmsrl(0x10, tsc);

    return tsc;
};
/*
 * Basically the same as arch/x86/kernel/irq.c --> "arch_irq_stat_cpu(cpu)"
 */
u64 sw_my_local_arch_irq_stats_cpu_i(void)
{
    u64 sum = 0;
    irq_cpustat_t *stats;
#ifdef __arm__
    int i=0;
#endif
    BEGIN_LOCAL_IRQ_STATS_READ(stats);
    {
#ifndef __arm__
        sum += stats->__nmi_count;
// #ifdef CONFIG_X86_LOCAL_APIC
        sum += stats->apic_timer_irqs;
// #endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        sum += stats->x86_platform_ipis;
#endif // 2,6,34
        sum += stats->apic_perf_irqs;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
        sum += stats->apic_irq_work_irqs;
#endif // 3,5,0
#ifdef CONFIG_SMP
        sum += stats->irq_call_count;
        sum += stats->irq_resched_count;
        sum += stats->irq_tlb_count;
#endif
#ifdef CONFIG_X86_THERMAL_VECTOR
        sum += stats->irq_thermal_count;
#endif
        sum += stats->irq_spurious_count; // should NEVER be non-zero!!!
#else
        sum += stats->__softirq_pending;
#ifdef CONFIG_SMP
        for (i=0; i<NR_IPI; ++i) {
            sum += stats->ipi_irqs[i];
        }
#endif
#ifdef CONFIG_X86_MCE
        sum += stats->mce_exception_count;
        sum += stats->mce_poll_count;
#endif
#endif
    }
    END_LOCAL_IRQ_STATS_READ(stats);
    return sum;
};

/*
 * Generic tracepoint/notifier handling function.
 */
void sw_handle_trace_notifier_i(struct sw_trace_notifier_data *node)
{
    struct sw_collector_data *curr = NULL;
    if (!node) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        sw_handle_per_cpu_msg(curr);
    }
};
/*
 * Generic tracepoint/notifier handling function.
 */
void sw_handle_trace_notifier_on_cpu_i(int cpu, struct sw_trace_notifier_data *node)
{
    struct sw_collector_data *curr = NULL;
    if (!node) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        sw_handle_per_cpu_msg_on_cpu(cpu, curr);
    }
};
/*
 * Tracepoint helpers.
 */
/*
 * IRQ wakeup handling function.
 */
void sw_handle_irq_wakeup_i(struct sw_collector_data *node, int irq)
{
    int cpu = RAW_CPU();
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    // char *dst_vals = (char *)(unsigned long)msg->p_payload;
    char *dst_vals = msg->p_payload;

    // msg->tsc = tscval(); // msg TSC assigned when msg is written to buffer
    msg->cpuidx = cpu;

    /*
     * IRQ handling ==> only return the irq number
     */
    *( (int *)dst_vals) = irq;

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
};
/*
 * TIMER wakeup handling funtion.
 */
void sw_handle_timer_wakeup_i(struct sw_collector_data *node, pid_t pid, pid_t tid)
{
    int cpu = RAW_CPU();
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    // char *dst_vals = (char *)(unsigned long)msg->p_payload;
    char *dst_vals = msg->p_payload;

    // msg->tsc = tscval(); // msg TSC assigned when msg is written to buffer
    msg->cpuidx = cpu;

    /*
     * TIMER handling ==> only return the pid, tid
     */
    *( (int *)dst_vals) = pid; dst_vals += sizeof(pid);
    *( (int *)dst_vals) = tid;

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
    pw_pr_debug("HANDLED timer expire for %d, %d\n", pid, tid);
};
/*
 * Helper function for {hr}timer expires. Required for overhead tracking.
 */
void sw_handle_timer_wakeup_helper_i(struct sw_collector_data *curr, struct sw_trace_notifier_data *node, pid_t tid)
{
    pid_t pid = -1;
    if (tid == 0) {
        pid = 0;
    } else {
        struct task_struct *task =  pid_task(find_pid_ns(tid, &init_pid_ns), PIDTYPE_PID);
        if (likely(task)) {
            pid = task->tgid;
        }
    }
    list_for_each_entry(curr, &node->list, list) {
        sw_handle_timer_wakeup_i(curr, pid, tid);
    }
};
/*
 * SCHED wakeup handling funtion.
 */
void sw_handle_sched_wakeup_i(struct sw_collector_data *node, int source_cpu, int target_cpu)
{
    int cpu = source_cpu;
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    // char *dst_vals = (char *)(unsigned long)msg->p_payload;
    char *dst_vals = msg->p_payload;

    // msg->tsc = tscval(); // msg TSC assigned when msg is written to buffer
    msg->cpuidx = source_cpu;

    /*
     * sched handling ==> only return the source, target CPUs
     */
    *( (int *)dst_vals) = source_cpu; dst_vals += sizeof(source_cpu);
    *( (int *)dst_vals) = target_cpu;

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_NONE)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
};
/*
 * APIC timer wakeup
 */
void sw_handle_apic_timer_wakeup_i(struct sw_collector_data *node)
{
    /*
     * Send an empty message back to Ring-3
     */
    int cpu = RAW_CPU();
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    // char *dst_vals = (char *)(unsigned long)msg->p_payload;

    // msg->tsc = tscval(); // msg TSC assigned when msg is written to buffer
    msg->cpuidx = cpu;

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
    pw_pr_debug("HANDLED APIC timer wakeup for cpu = %d\n", cpu);
};
/*
 * Helper function for workqueue executions. Required for overhead tracking.
 */
void sw_handle_workqueue_wakeup_helper_i(int cpu, struct sw_collector_data *node)
{
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);

    // msg->tsc = tscval(); // msg TSC assigned when msg is written to buffer
    msg->cpuidx = cpu;

    /*
     * Workqueue wakeup ==> empty message.
     */
    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_error("WARNING: could NOT produce message!\n");
    }
};
/*
 * Helper function for sched_switch. Required for overhead tracking.
 */
void sw_handle_sched_switch_helper_i(void)
{
    static struct sw_trace_notifier_data *node = NULL;
    unsigned long delta_msecs = 0x0;
    unsigned long curr_jiff = jiffies, prev_jiff = __get_cpu_var(sw_pcpu_polling_jiff);
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_SWITCH);
        pw_pr_debug("SCHED SWITCH NODE = %p\n", node);
    }
    /*
     * Has it been enough time since the last collection point?
     */
    delta_msecs = jiffies_to_msecs(curr_jiff) - jiffies_to_msecs(prev_jiff);
    if (delta_msecs < sw_min_polling_interval_msecs) {
        // NO!
        return;
    }
    __get_cpu_var(sw_pcpu_polling_jiff) = curr_jiff;
    preempt_disable();
    {
        struct sw_collector_data *curr = NULL;
        list_for_each_entry(curr, &node->list, list) {
            /*
             * Did the user ask to run on this CPU?
             * Update: or 'ANY' CPU?
             */
            struct cpumask *mask = &curr->cpumask;
            if (cpumask_test_cpu(RAW_CPU(), mask)) {
                // User wants to run on THIS cpu
                sw_handle_per_cpu_msg_no_sched(curr);
            } else if (cpumask_empty(mask)) {
                // User wants to run on ANY cpu
                unsigned long prev_jiffs = curr->last_update_jiffies, curr_jiffs = jiffies;
                unsigned long delta_msecs = jiffies_to_msecs(curr_jiffs) - jiffies_to_msecs(prev_jiffs);
                /*
                 * Has it been enough time since the last collection point?
                 * We need this additional check here because nodes with empty
                 * cpu masks can potentially be handled by ANY cpu, so it's possible
                 * this node has been recently serviced by a different CPU.
                 */
                if (delta_msecs < sw_min_polling_interval_msecs) {
                    continue;
                }
                /*
                 * CAS is required for double-checked locking.
                 */
                if (CAS64(&curr->last_update_jiffies, prev_jiffs, curr_jiffs) == false) {
                    /*
                     * Somebody handled this already.
                     */
                    continue;
                }
                sw_handle_per_cpu_msg_no_sched(curr);
            }
        }
    }
    preempt_enable();
};

/*
 * Probe functions.
 */
/*
 * 1. TPS
 */
/*
 * Check IPI wakeups within the cpu_idle tracepoint.
 */
void sw_tps_apic_i(int cpu)
{
    static struct sw_trace_notifier_data *apic_timer_node = NULL;
    if (unlikely(apic_timer_node == NULL)) {
        apic_timer_node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_IPI);
        pw_pr_debug("apic NODE = %p\n", apic_timer_node);
    }
    if (apic_timer_node) {
        bool local_apic_timer_fired = false;
        u64 curr_num_local_apic = sw_my_local_arch_irq_stats_cpu_i();
        u64 *old_num_local_apic = &__get_cpu_var(sw_num_local_apic_timer_inters);

        if (*old_num_local_apic && (*old_num_local_apic != curr_num_local_apic)) {
            local_apic_timer_fired = true;
        }
        *old_num_local_apic = curr_num_local_apic;

        if (local_apic_timer_fired && SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
            struct sw_collector_data *curr = NULL;
            list_for_each_entry(curr, &apic_timer_node->list, list) {
                sw_handle_apic_timer_wakeup_i(curr);
            }
        }
    }
};
/*
 * Perform any user-defined tasks within the
 * cpu_idle tracepoint.
 */
void sw_tps_tps_i(int cpu)
{
    static struct sw_trace_notifier_data *tps_node = NULL;
    if (unlikely(tps_node == NULL)) {
        tps_node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_CPU_IDLE);
        pw_pr_debug("TPS NODE = %p\n", tps_node);
    }
    sw_handle_trace_notifier_i(tps_node);
};
/*
 * Perform any wakeup-related tasks within the
 * cpu_idle tracepoint.
 */
void sw_tps_wakeup_i(int cpu)
{
    /*
     * For now, assume we will always have to
     * do some wakeup book keeping. Later, we'll
     * need to detect if the user requested wakeups.
     */
    sw_wakeup_event_flag  = false;
    RESET_VALID_WAKEUP_EVENT_COUNTER(cpu);
};
void sw_tps_i(void)
{
    /*
     * Update: FIRST handle IPI wakeups
     * THEN handle TPS
     */
    int cpu = RAW_CPU();
    sw_tps_apic_i(cpu);
    sw_tps_tps_i(cpu);
    sw_tps_wakeup_i(cpu);
};

DEFINE_PROBE_FUNCTION(PROBE_TPS_PARAMS)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
    if (state == PWR_EVENT_EXIT) {
        return;
    }
#endif
    DO_PER_CPU_OVERHEAD_FUNC(sw_tps_i);
};

/*
 * 2. TPF
 */
/*
 * Helper function for overhead measurements.
 */
void sw_tpf_i(int cpu, struct sw_trace_notifier_data *node)
{
    sw_handle_trace_notifier_on_cpu_i((int)cpu, node);
};

DEFINE_PROBE_FUNCTION(PROBE_TPF_PARAMS)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    int cpu = RAW_CPU();
#endif // version < 2.6.38
    static struct sw_trace_notifier_data *node = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_CPU_FREQUENCY);
        pw_pr_debug("NODE = %p\n", node);
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};

/*
 * 3. IRQ handler entry
 */
DEFINE_PROBE_FUNCTION(PROBE_IRQ_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_IRQ_HANDLER_ENTRY);
        pw_pr_debug("NODE = %p\n", node);
    }
    if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        DO_PER_CPU_OVERHEAD_FUNC(sw_handle_irq_wakeup_i, curr, irq);
    }
};
/*
 * 4. TIMER expire
 */
DEFINE_PROBE_FUNCTION(PROBE_TIMER_ARGS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    pid_t tid = GET_TIMER_THREAD_ID(t);

    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_TIMER_EXPIRE_ENTRY);
        pw_pr_debug("NODE = %p\n", node);
    }

    if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
        return;
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_handle_timer_wakeup_helper_i, curr, node, tid);
};
/*
 * 5. HRTIMER expire
 */
DEFINE_PROBE_FUNCTION(PROBE_HRTIMER_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    pid_t tid = GET_TIMER_THREAD_ID(hrt);

    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY);
        pw_pr_debug("NODE = %p\n", node);
    }

    if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
        return;
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_handle_timer_wakeup_helper_i, curr, node, tid);
};
/*
 * 6. SCHED wakeup
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_WAKEUP_PARAMS)
{
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    int target_cpu = task_cpu(task), source_cpu = RAW_CPU();
    /*
     * "Self-sched" samples are "don't care".
     */
    if (target_cpu == source_cpu) {
        return;
    }
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_WAKEUP);
        pw_pr_debug("NODE = %p\n", node);
    }
    /*
     * Unlike other wakeup sources, we check the per-cpu flag
     * of the TARGET cpu to decide if we should produce a sample.
     */
    if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(target_cpu)) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        // sw_handle_sched_wakeup_i(curr, source_cpu, target_cpu);
        DO_PER_CPU_OVERHEAD_FUNC(sw_handle_sched_wakeup_i, curr, source_cpu, target_cpu);
    }
};
/*
 * 8. PROCESS fork
 */
/*
 * Helper for PROCESS fork, PROCESS exit
 */
void sw_process_fork_exit_helper_i(struct sw_collector_data *node, struct task_struct *task, bool is_fork)
{
    int cpu = RAW_CPU();
    pid_t pid = task->tgid, tid = task->pid;
    const char *name = task->comm;
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    char *dst_vals = msg->p_payload;

    msg->cpuidx = cpu;

    /*
     * Fork/Exit ==> return pid, tid
     * Fork ==> also return name
     */
    *( (int *)dst_vals) = pid; dst_vals += sizeof(pid);
    *( (int *)dst_vals) = tid; dst_vals += sizeof(tid);
    if (is_fork) {
        memcpy(dst_vals, name, SW_MAX_PROC_NAME_SIZE);
    }

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
    pw_pr_debug("HANDLED process %s event for task: pid = %d, tid = %d, name = %s\n", is_fork ? "FORK" : "EXIT", pid, tid, name);
};

DEFINE_PROBE_FUNCTION(PROBE_PROCESS_FORK_PARAMS)
{
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_PROCESS_FORK);
        pw_pr_debug("NODE = %p\n", node);
    }
    if (!node) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        DO_PER_CPU_OVERHEAD_FUNC(sw_process_fork_exit_helper_i, curr, child, true /* true ==> fork */);
    }
};
/*
 * 9. PROCESS exit
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_PROCESS_EXIT_PARAMS)
{
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_PROCESS_EXIT);
        pw_pr_debug("NODE = %p\n", node);
    }
    if (!node) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        DO_PER_CPU_OVERHEAD_FUNC(sw_process_fork_exit_helper_i, curr, task, false /* false ==> exit */);
    }
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
/*
 * 10. THERMAL_APIC entry
 */
DEFINE_PROBE_FUNCTION(PROBE_THERMAL_APIC_ENTRY_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_THERMAL_APIC_ENTRY);
        pw_pr_debug("NODE = %p\n", node);
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};
/*
 * 10. THERMAL_APIC exit
 */
DEFINE_PROBE_FUNCTION(PROBE_THERMAL_APIC_EXIT_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_THERMAL_APIC_EXIT);
        pw_pr_debug("NODE = %p\n", node);
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)

#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
/*
 * 11. WAKE lock / WAKEUP source activate.
 */
/*
 * Helper function to produce wake lock/unlock messages.
 */
void sw_produce_wakelock_msg_i(int cpu, struct sw_collector_data *node,
			       const char *name, int type, u64 timeout,
			       int pid, int tid, const char *proc_name)
{
    sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu, node->per_msg_payload_size);
    char *dst_vals = msg->p_payload;

    msg->cpuidx = cpu;

    /*
     * Protocol:
     * wakelock_timeout, wakelock_type, wakelock_name, proc_pid, proc_tid, proc_name
     */
    *((u64 *)dst_vals) = timeout; dst_vals += sizeof(timeout);
    *((int *)dst_vals) = type; dst_vals += sizeof(type);
    strncpy(dst_vals, name, SW_MAX_KERNEL_WAKELOCK_NAME_SIZE); dst_vals += SW_MAX_KERNEL_WAKELOCK_NAME_SIZE;

    *((int *)dst_vals) = pid; dst_vals += sizeof(pid);
    *((int *)dst_vals) = tid; dst_vals += sizeof(tid);
    strncpy(dst_vals, proc_name, SW_MAX_PROC_NAME_SIZE); dst_vals += SW_MAX_PROC_NAME_SIZE;

    if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT)) {
        pw_pr_warn("WARNING: could NOT produce message!\n");
    }
};
/*
 * Helper function to handle wake lock/unlock callbacks.
 */
void sw_handle_wakelock_i(int cpu, struct sw_trace_notifier_data *node, const char *name, int type, u64 timeout)
{
    int pid = PID(), tid = TID();
    const char *proc_name = NAME();
    struct sw_collector_data *curr = NULL;

    if (!node) {
        return;
    }

    list_for_each_entry(curr, &node->list, list) {
        sw_produce_wakelock_msg_i(cpu, curr, name, type, timeout, pid, tid, proc_name);
    }
};
DEFINE_PROBE_FUNCTION(PROBE_WAKE_LOCK_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    enum sw_kernel_wakelock_type type = SW_WAKE_LOCK;
    u64 timeout = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
    const char *name = lock->name;
#endif

    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_WAKE_LOCK);
        pw_pr_debug("NODE = %p\n", node);
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
    /*
     * Was this wakelock acquired with a timeout i.e.
     * is this an auto expire wakelock?
     */
    if (lock->flags & (1U << 10)) {
        type = SW_WAKE_LOCK_TIMEOUT;
        timeout = jiffies_to_msecs(lock->expires - jiffies);
    }
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
    DO_PER_CPU_OVERHEAD_FUNC(sw_handle_wakelock_i, cpu, node, name, (int)type, timeout);
};
/*
 * 11. WAKE unlock / WAKEUP source deactivate.
 */
DEFINE_PROBE_FUNCTION(PROBE_WAKE_UNLOCK_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    enum sw_kernel_wakelock_type type = SW_WAKE_UNLOCK;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
    const char *name = lock->name;
#endif

    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_WAKE_UNLOCK);
        pw_pr_debug("NODE = %p\n", node);
    }
    DO_PER_CPU_OVERHEAD_FUNC(sw_handle_wakelock_i, cpu, node, name, (int)type, 0 /*timeout*/);
};
#endif // CONFIG_SOCWATCH_ANDROID

/*
 * 12. WORKQUEUE
 */
DEFINE_PROBE_FUNCTION(PROBE_WORKQUEUE_PARAMS)
{
    int cpu = RAW_CPU();
    static struct sw_trace_notifier_data *node = NULL;
    struct sw_collector_data *curr = NULL;

    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_WORKQUEUE_EXECUTE_START);
        pw_pr_debug("NODE = %p\n", node);
    }

    if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
        return;
    }
    list_for_each_entry(curr, &node->list, list) {
        DO_PER_CPU_OVERHEAD_FUNC(sw_handle_workqueue_wakeup_helper_i, cpu, curr);
    }
};

/*
 * 13. SCHED switch
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_SWITCH_PARAMS)
{
    DO_PER_CPU_OVERHEAD_FUNC(sw_handle_sched_switch_helper_i);
};

/*
 * 1. SUSPEND notifier
 */
int sw_probe_suspend_notifier_i(struct notifier_block *block, unsigned long state, void *dummy)
{
    static struct sw_trace_notifier_data *node = NULL;
    if (unlikely(node == NULL)) {
        node = GET_COLLECTOR_NOTIFIER_NODE(SW_NOTIFIER_ID_SUSPEND_NOTIFIER);
        pw_pr_debug("NODE = %p\n", node);
    }
    switch (state) {
        case PM_SUSPEND_PREPARE:
            /*
             * Entering SUSPEND.
             */
        case PM_POST_SUSPEND: // Fall-through
            /*
             * Exitted SUSPEND.
             * For now, don't send the 'state' parameter
             * back to Ring-3
             */
            sw_handle_trace_notifier_i(node);
            break;
        default:
            pw_pr_error("ERROR: unknown state %lu passed to SWA suspend notifier!\n", state);
            break;
    }
    return NOTIFY_DONE;
};

/*
 * 1. TPS.
 */
int sw_register_trace_cpu_idle_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, power_start, sw_probe_power_start_i);
#else // kernel version >= 2.6.38
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, cpu_idle, sw_probe_cpu_idle_i);
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    return PW_SUCCESS;
};
int sw_unregister_trace_cpu_idle_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, power_start, sw_probe_power_start_i);
#else // kernel version >= 2.6.38
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, cpu_idle, sw_probe_cpu_idle_i);
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    return PW_SUCCESS;
};
/*
 * 2. TPF
 */
int sw_register_trace_cpu_frequency_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, power_frequency, sw_probe_power_frequency_i);
#else // kernel version >= 2.6.38
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, cpu_frequency, sw_probe_cpu_frequency_i);
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    return PW_SUCCESS;
};
int sw_unregister_trace_cpu_frequency_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, power_frequency, sw_probe_power_frequency_i);
#else // kernel version >= 2.6.38
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, cpu_frequency, sw_probe_cpu_frequency_i);
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
    return PW_SUCCESS;
};
/*
 * 3. IRQ handler entry
 */
int sw_register_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, irq_handler_entry, sw_probe_irq_handler_entry_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, irq_handler_entry, sw_probe_irq_handler_entry_i);
    return PW_SUCCESS;
};
/*
 * 4. TIMER expire.
 */
int sw_register_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, timer_expire_entry, sw_probe_timer_expire_entry_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, timer_expire_entry, sw_probe_timer_expire_entry_i);
    return PW_SUCCESS;
};
/*
 * 5. HRTIMER expire.
 */
int sw_register_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, hrtimer_expire_entry, sw_probe_hrtimer_expire_entry_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, hrtimer_expire_entry, sw_probe_hrtimer_expire_entry_i);
    return PW_SUCCESS;
};
/*
 * 6. SCHED wakeup
 */
int sw_register_trace_sched_wakeup_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_wakeup, sw_probe_sched_wakeup_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_sched_wakeup_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_wakeup, sw_probe_sched_wakeup_i);
    return PW_SUCCESS;
};
/*
 * 8. PROCESS fork
 */
int sw_register_trace_sched_process_fork_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_process_fork, sw_probe_sched_process_fork_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_sched_process_fork_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_process_fork, sw_probe_sched_process_fork_i);
    return PW_SUCCESS;
};
/*
 * 9. PROCESS exit
 */
int sw_register_trace_sched_process_exit_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_process_exit, sw_probe_sched_process_exit_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_sched_process_exit_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_process_exit, sw_probe_sched_process_exit_i);
    return PW_SUCCESS;
};
/*
 * 10. THERMAL_APIC entry
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
int sw_register_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_entry, sw_probe_thermal_apic_entry_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_entry, sw_probe_thermal_apic_entry_i);
    return PW_SUCCESS;
};
/*
 * 10. THERMAL_APIC exit
 */
int sw_register_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_exit, sw_probe_thermal_apic_exit_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_exit, sw_probe_thermal_apic_exit_i);
    return PW_SUCCESS;
};
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
/*
 * 11. WAKE lock / WAKEUP source activate.
 */
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
int sw_register_trace_wake_lock_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, wake_lock, sw_probe_wake_lock_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_wake_lock_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wake_lock, sw_probe_wake_lock_i);
    return PW_SUCCESS;
};
#else // LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
int sw_register_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_activate, sw_probe_wakeup_source_activate_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_activate, sw_probe_wakeup_source_activate_i);
    return PW_SUCCESS;
};
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
/*
 * 11. WAKE unlock / WAKEUP source deactivate.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
int sw_register_trace_wake_unlock_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, wake_unlock, sw_probe_wake_unlock_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_wake_unlock_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wake_unlock, sw_probe_wake_unlock_i);
    return PW_SUCCESS;
};
#else // LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
int sw_register_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node)
{
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_deactivate, sw_probe_wakeup_source_deactivate_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_deactivate, sw_probe_wakeup_source_deactivate_i);
    return PW_SUCCESS;
};
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#endif // CONFIG_SOCWATCH_ANDROID
/*
 * 12. WORKQUEUE execution.
 */
int sw_register_trace_workqueue_execution_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execution, sw_probe_workqueue_execution_i);
#else
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execute_start, sw_probe_workqueue_execute_start_i);
#endif
    return PW_SUCCESS;
};
int sw_unregister_trace_workqueue_execution_i(struct sw_trace_notifier_data *node)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execution, sw_probe_workqueue_execution_i);
#else
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execute_start, sw_probe_workqueue_execute_start_i);
#endif
    return PW_SUCCESS;
};
/*
 * 13. SCHED switch
 */
int sw_register_trace_sched_switch_i(struct sw_trace_notifier_data *node)
{
    /*
     * Set polling tick time, in jiffies.
     * Used by the context switch tracepoint to decide
     * if enough time has elapsed since the last
     * collection point to read resources again.
     */
    {
        int cpu = 0;
        for_each_possible_cpu(cpu) {
            *(&per_cpu(sw_pcpu_polling_jiff, cpu)) = jiffies;
        }
    }
    DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_switch, sw_probe_sched_switch_i);
    return PW_SUCCESS;
};
int sw_unregister_trace_sched_switch_i(struct sw_trace_notifier_data *node)
{
    DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_switch, sw_probe_sched_switch_i);
    return PW_SUCCESS;
};
/*
 * Notifier register/unregister functions.
 */
/*
 * 1. SUSPEND notifier.
 */
struct notifier_block sw_pm_suspend_notifier = {
    .notifier_call = &sw_probe_suspend_notifier_i,
};
int sw_register_suspend_notifier_i(struct sw_trace_notifier_data *node)
{
    register_pm_notifier(&sw_pm_suspend_notifier);
    return PW_SUCCESS;
};
int sw_unregister_suspend_notifier_i(struct sw_trace_notifier_data *node)
{
    unregister_pm_notifier(&sw_pm_suspend_notifier);
    return PW_SUCCESS;
};

/*
 * Tracepoint extraction routines.
 * Required for newer kernels (>=3.15)
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
static void sw_extract_tracepoint_callback(struct tracepoint *tp, void *priv)
{
    struct sw_trace_notifier_data *node = NULL;
    int i=0;
    int *numStructsFound = (int *)priv;
    if (*numStructsFound == NUM_VALID_TRACEPOINTS) {
        /*
         * We've found all the tracepoints we need.
         */
        return;
    }
    if (tp) {
        FOR_EACH_TRACEPOINT_NODE(i, node) {
            if (node->tp == NULL && node->name) {
                const char *name = sw_get_trace_notifier_kernel_name(node);
                if (name && !strcmp(tp->name, name)) {
                    node->tp = tp;
                    ++*numStructsFound;
                    pw_pr_debug("OK, found TP %s\n", tp->name);
                }
            }
        }
    }
};
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)

/*
 * Retrieve the list of tracepoint structs to use when registering and unregistering
 * tracepoint handlers.
 */
int sw_extract_trace_notifier_providers(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
    int numCallbacks = 0;
    for_each_kernel_tracepoint(&sw_extract_tracepoint_callback, &numCallbacks);
    /*
     * Did we get the complete list?
     */
    if (numCallbacks != NUM_VALID_TRACEPOINTS) {
        pw_pr_error("ERROR: Could NOT find tracepoint structs for some required tracepoints!\n");
        return -PW_ERROR;
    }
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
    return PW_SUCCESS;
};

void sw_reset_trace_notifier_providers(void)
{
    /*
     * Reset the wakeup flag. Not strictly required if we aren't probing
     * any of the wakeup tracepoints.
     */
    {
        int cpu = 0;
        for_each_online_cpu(cpu) {
            RESET_VALID_WAKEUP_EVENT_COUNTER(cpu);
        }
    }
    /*
     * Reset the wakeup event flag. Not strictly required if we
     * aren't probing any of the wakeup tracepoints. Will be reset
     * in the power_start tracepoint if user requested a c-state
     * collection.
     */
    sw_wakeup_event_flag = true;
};

void sw_print_trace_notifier_provider_overheads(void)
{
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_tps_i, "TPS");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_tpf_i, "TPF");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_irq_wakeup_i, "IRQ");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_timer_wakeup_helper_i, "TIMER_EXPIRE");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_sched_wakeup_i, "SCHED WAKEUP");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_process_fork_exit_helper_i, "PROCESS FORK/EXIT");
#if IS_ENABLED(CONFIG_SOCWATCH_ANDROID)
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_wakelock_i, "WAKE LOCK/UNLOCK");
#endif // CONFIG_SOCWATCH_ANDROID
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_workqueue_wakeup_helper_i, "WORKQUEUE");
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_sched_switch_helper_i, "SCHED SWITCH");
};
/*
 * Add all trace/notifier providers.
 */
int sw_add_trace_notifier_providers(void)
{
    struct sw_trace_notifier_data *node = NULL;
    int i = 0;
    FOR_EACH_TRACEPOINT_NODE(i, node) {
        if (sw_register_trace_notify_provider(node)) {
            pw_pr_error("ERROR: couldn't add a trace provider!\n");
            return -EIO;
        }
    }
    FOR_EACH_NOTIFIER_NODE(i, node) {
        if (sw_register_trace_notify_provider(node)) {
            pw_pr_error("ERROR: couldn't add a notifier provider!\n");
            return -EIO;
        }
    }
    return PW_SUCCESS;
}
/*
 * Remove previously added providers.
 */
void sw_remove_trace_notifier_providers(void)
{
    // NOP
}
