#ifndef __SW_DATA_STRUCTS_H__
#define __SW_DATA_STRUCTS_H__

/*
 * Taken from 'sw_driver'
 * TODO: move to separate file?
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>		// inode
#include <linux/device.h>	// class_create
#include <linux/cdev.h>		// cdev_alloc
#include <linux/uaccess.h>	// copy_to_user
#include <linux/vmalloc.h>	// vmalloc
#include <linux/sched.h>	// TASK_INTERRUPTIBLE
#include <linux/wait.h>		// wait_event_interruptible
#include <linux/pci.h>		// pci_get_bus_and_slot
#include <linux/sfi.h>		// For SFI F/W version
#include <asm/hardirq.h>
#include <linux/cpufreq.h>
#include <asm/local.h>		// local_t
#include <linux/hardirq.h>	// "in_atomic"

#ifdef CONFIG_X86_WANT_INTEL_MID
#include <asm/intel-mid.h>
#endif // CONFIG_X86_WANT_INTEL_MID
/*
 * End taken from sw_driver
 */

#include "sw_structs.h"
#include "sw_ioctl.h"
#include "sw_list.h"

/* ******************************************
 * Compile time constants
 * ******************************************
 */
#define GET_POLLED_CPU() (sw_max_num_cpus)

/* ******************************************
 * Function declarations.
 * ******************************************
 */
/*
 * Output to user.
 */
unsigned long sw_copy_to_user(void *dst, char *src, size_t bytes_to_copy);
bool sw_check_output_buffer_params(void *buffer, size_t bytes_to_read,
				   size_t buff_size);
/*
 * smp call function.
 */
void sw_schedule_work(const struct cpumask *mask, void (*work) (void *),
		      void *data);
/*
 * Save IRQ flags and retrieve cpu number.
 */
int sw_get_cpu(unsigned long *flags);
/*
 * Restore IRQ flags.
 */
void sw_put_cpu(unsigned long flags);
/*
 * Set module scope for cpu frequencies.
 */
int sw_set_module_scope_for_cpus(void);
/*
 * reset module scope for cpu frequencies.
 */
int sw_reset_module_scope_for_cpus(void);

#endif // __SW_DATA_STRUCTS_H__
