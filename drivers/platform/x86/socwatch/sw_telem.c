/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2017 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2017 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/compiler.h>     /* Definition of __weak */
#include "sw_defines.h"         /* PW_ERROR, etc. */
#include "sw_kernel_defines.h"  /* pw_pr_debug */
#include "sw_mem.h"             /* sw_kmalloc/free */
#include "sw_telem.h"           /* Signatures of fn's exported from here. */

/*
 * These functions and data structures are exported by the Telemetry
 * driver.  However, that file may not be available in the kernel for
 * which this driver is being built, so we re-define many of the same
 * things here.
 */
/**
 * struct telemetry_evtlog - The "event log" returned by the kernel's
 *                           full-read telemetry driver.
 * @telem_evtid:   The 16-bit event ID.
 * @telem_evtlog:  The actual telemetry data.
 */
struct telemetry_evtlog {
    u32 telem_evtid;    /* Event ID of a data item. */
    u64 telem_evtlog;   /* Counter data */
};

struct telemetry_evtconfig {
    u32 *evtmap;    /* Array of Event-IDs to Enable */
    u8 num_evts;    /* Number of Events (<29) in evtmap */
    u8 period;      /* Sampling period */
};

#define MAX_TELEM_EVENTS 28  /* Max telem events per unit */

/* The enable bit is set when programming events, but is returned
 * cleared for queried events requests.
 */
#define TELEM_EVENT_ENABLE 0x8000 /* Enabled when Event ID HIGH bit */

/*
 * Sampling Period values.
 * The sampling period is encoded in an 7-bit value, where
 *    Period = (Value * 16^Exponent) usec where:
 *        bits[6:3] -> Value;
 *        bits [0:2]-> Exponent;
 * Here are some of the calculated possible values:
 * | Value  Val+Exp  | Value | Exponent | Period (usec) | Period (msec) |
 * |-----------------+-------+----------+---------------+---------------|
 * | 0xA = 000 1+010 |     1 |        2 |           256 |         0.256 |
 * | 0x12= 001 0+010 |     2 |        2 |           512 |         0.512 |
 * | 0x22= 010 0+010 |     4 |        2 |          1024 |         1.024 |
 * | 0xB = 000 1+011 |     1 |        3 |          4096 |         4.096 |
 * | 0x13= 001 0+011 |     2 |        3 |          8192 |         8.192 |
 * | 0x1B= 001 1+011 |     3 |        3 |         12288 |        12.288 |
 * | 0x0C= 000 1+100 |     1 |        4 |         65536 |        65.536 |
 * | 0x0D= 000 1+101 |     1 |        5 |       1048576 |      1048.576 |
 */
#define TELEM_SAMPLING_1MS 0x22  /* Approximately 1 ms */
#define TELEM_SAMPLING_1S  0x0D  /* Approximately 1 s */

/* These functions make up the main APIs of the telemetry driver.  We
 * define all of them with weak linkage so that we can still compile
 * and load into kernels which don't have a telemetry driver.
 */
extern int __weak telemetry_raw_read_eventlog(enum telemetry_unit telem_unit,
                                              struct telemetry_evtlog *evtlog,
                                              int evcount);
extern int __weak telemetry_reset(void);
extern int __weak telemetry_reset_events(void);
extern int __weak telemetry_get_sampling_period(u8 *punit_min,
                                                u8 *punit_max,
                                                u8 *pmc_min,
                                                u8 *pmc_max);
extern int __weak telemetry_set_sampling_period(u8 punit_period,
                                                u8   pmc_period);
extern int __weak telemetry_get_eventconfig(struct telemetry_evtconfig *punit_config,
                                            struct telemetry_evtconfig *pmc_config,
                                            int  punit_len,
                                            int  pmc_len);
extern int __weak telemetry_add_events(u8 num_punit_evts, u8 num_pmc_evts,
                                       u32 *punit_evtmap, u32 *pmc_evtmap);

/*
 * Some telemetry IDs have multiple instances, indexed by cpu ID.  We
 * implement these by defining two types of IDs: 'regular' and 'scaled'.
 * For Telemetry IDs with a single instance (the majority of them), the
 * index into the system's telemetry table is stored in the
 * sw_driver_io_descriptor.idx.  At read time, the driver gets the telemetry
 * "slot" from sw_driver_io_descriptor.idx, and reads that data.  This case
 * is illustrated by telem_desc_A in the illustration below, where idx 2
 * indicates that telem_data[2] contains the telem data for this descriptor.
 *
 *   telem_desc_A                            telem_data
 *    scale_op: X                              |..|[0]
 *    idx     : 2 --------------------         |..|[1]
 *                                    \------->|..|[2]
 *                     Scaled_IDs              |..|[3]
 *   telem_desc_B     CPU#0 1 2 3       ------>|..|[4]
 *    scale_op: /     [0]|.|.|.|.|     /
 *    idx     : 1---->[1]|4|4|5|5|    /
 *                        +----------/
 *
 * Descriptors with scaled IDs contain a scale operation (scale_op) and
 * value.  They use a 'scaled_ids' table, which is indexed by descriptor
 * number and CPU id, and stores the telem_data index.  So in the
 * illustration above, CPU 0 reading from telem_desc_B would fetch row 1
 * (from telem_desc_B.idx == 1), and column [0] yielding element 4, so
 * that's the telemetry ID it looks up in the telemetry data.
 *
 * The scaled_ids table is populated at telemetry ID initialization time
 *
 */
static unsigned char *sw_telem_scaled_ids = NULL; /* Allocate on demand */
static unsigned int   sw_telem_rows_alloced = 0; /* Rows currently allocated */
static unsigned int   sw_telem_rows_avail   = 0; /* Available rows */

extern int sw_max_num_cpus;     /* SoC Watch's copy of cpu count. */


/* Macro for identifying telemetry IDs with either per-cpu, or per-module
 * instances.  These IDs need to be 'scaled' as per scale_op and scale_val.
 */
#define IS_SCALED_ID(td) ((td)->scale_op != TELEM_OP_NONE)


/**
 * telemetry_available - Determine if telemetry driver is present
 *
 * Returns: 1 if telemetry driver is present, 0 if not.
 */
static int telemetry_available(void)
{
    int retval = 0;
    struct telemetry_evtconfig punit_evtconfig;
    struct telemetry_evtconfig pmc_evtconfig;
    u32 punit_event_map[MAX_TELEM_EVENTS];
    u32 pmc_event_map[MAX_TELEM_EVENTS];

    /* The symbol below is weak.  We return 1 if we have a definition
     * for this telemetry-driver-supplied symbol, or 0 if only the
     * weak definition exists. This test will suffice to detect if
     * the telemetry driver is loaded.
     */
    if (telemetry_get_eventconfig == 0) {
            return 0;
    }
    /* OK, the telemetry driver is loaded. But it's possible it
     * hasn't been configured properly. To check that, retrieve
     * the number of events currently configured. This should never
     * be zero since the telemetry driver reserves some SSRAM slots
     * for its own use
     */
    memset(&punit_evtconfig, 0, sizeof(punit_evtconfig));
    memset(&pmc_evtconfig, 0, sizeof(pmc_evtconfig));

    punit_evtconfig.evtmap = (u32*) &punit_event_map;
    pmc_evtconfig.evtmap = (u32*) &pmc_event_map;

    retval = telemetry_get_eventconfig(&punit_evtconfig, &pmc_evtconfig,
                                        MAX_TELEM_EVENTS, MAX_TELEM_EVENTS);
    return retval == 0 && punit_evtconfig.num_evts > 0 && pmc_evtconfig.num_evts > 0;
}



/**
 * sw_telem_find_event - Find event id in the array of events.
 *
 * @id:       Event ID to search for.
 * @events:   Array of events IDs to search through.
 * @ecount:   Number of elements in the events array.
 *
 * Returns:   -1 if id not found, array offset if it is found.
 */
static int sw_telem_find_event(u32 id, const u32 *events,
                               unsigned int ecount)
{
    unsigned int i;
    id &= ~TELEM_EVENT_ENABLE;  /* Returned events have enable cleared */
    for (i = 0; i < ecount; i++) {
        if ((events[i] & ~TELEM_EVENT_ENABLE) == id) {
            return i;
        }
    }
    return -1;
}


/**
 * sw_telem_register_id - Register a telemetry event ID to be collected.
 * Returns: The Index of the recorded event (if successful)
 *          -PW_ERROR on failure.
 */
static int sw_telem_register_id(unsigned int unit, unsigned int id)
{
    struct telemetry_evtconfig punit_evtconfig;
    struct telemetry_evtconfig pmc_evtconfig;
    struct telemetry_evtconfig *unit_evtconfig;
    u32 punit_event_map[MAX_TELEM_EVENTS];
    u32 pmc_event_map[MAX_TELEM_EVENTS];
    int idx;  /* Index into telemetry data array of event ID to gather. */
    int retval;

    if (unit == TELEM_PUNIT) {
        unit_evtconfig = &punit_evtconfig;
    } else if (unit == TELEM_PMC) {
        unit_evtconfig = &pmc_evtconfig;
    } else {
        pw_pr_error("ERROR: TELEM unknown unit: %d\n", unit);
        return -PW_ERROR;
    }

    memset(unit_evtconfig, 0, sizeof(*unit_evtconfig));

    /* Initialize the event maps within evtconfig struct */
    punit_evtconfig.evtmap = (u32*) &punit_event_map;
    pmc_evtconfig.evtmap = (u32*) &pmc_event_map;

    /* Read the events from the telemetry unit, and see if the event we
     * want is already being gathered.
     */
    retval = telemetry_get_eventconfig(&punit_evtconfig, &pmc_evtconfig,
                                        MAX_TELEM_EVENTS, MAX_TELEM_EVENTS);
    if (retval != PW_SUCCESS) {
        pw_pr_error("ERROR: failure retrieving events during event: 0x%x "
                    "add -- Ignoring event", id);
        return -PW_ERROR;
    }

    idx = sw_telem_find_event(id, unit_evtconfig->evtmap, unit_evtconfig->num_evts);
    if (idx == -1) {
        /* Event is not tracked by default; add it to the custom ID set. */
        pw_pr_debug("sw_telem_init_func(): metric id %x not found--adding\n",
                    id);
        if (telemetry_add_events((unit == TELEM_PUNIT), (unit == TELEM_PMC),
                                 &id, &id) != PW_SUCCESS) {
            pw_pr_error("ERROR: could not program telem event ID: 0x%x "
                        "-- Ignoring event", id);
            return -PW_ERROR;
        }
        pw_pr_debug("sw_telem_init:(): post-add: punit: evts: %d, "
                    "pmc: evts: %d\n", event_count[0], event_count[1]);

        /* Now see if we can find the event's index. */
        telemetry_get_eventconfig(&punit_evtconfig, &pmc_evtconfig,
                                   MAX_TELEM_EVENTS, MAX_TELEM_EVENTS);

        idx = sw_telem_find_event(id, unit_evtconfig->evtmap, unit_evtconfig->num_evts);
        if (idx == -1) {
            pw_pr_error("ERROR: TELEM added event not found! ID=0x%x\n",
                        id);
            return -PW_ERROR;   /* The event wasn't added successfuly. */
        }
    }
    return idx;
}

/**
 * sw_get_instance_row -- Get the address of a 'row' of instance IDs.
 * @rownum: The row number of the Instance ID table, whose address to return.
 * Returns: The address of the appropriate row, or NULL if rownum is bad.
 */
static unsigned char *sw_get_instance_row_addr(unsigned int rownum)
{
    if (rownum >= (sw_telem_rows_alloced - sw_telem_rows_avail)) {
        pw_pr_error("ERROR: Cannot retrieve row Instance ID row %d\n",
                    rownum);
        return NULL;
    }
    return &sw_telem_scaled_ids[rownum * sw_max_num_cpus];
}

/**
 * sw_telem_get_scaled_tbl_row - Get the row number of a free table row.
 * Returns: index of a newly-allocated row in the table (if successful).
 *          -1 if realloc fails.
 *
 * Check if there is an available row in the instance ID table.  If not,
 * effectively realloc the current table (allocate new storage, and move
 * the existing data into the new table.)  Note that rows are allocated
 * in chunks in an attempt to reduce data movement.
 *
 * Since IDs are only requested during the init phase, we don't need
 * to worry about providing mutual exclusion or locking the table,
 * during reallocation.
 */
static int sw_telem_get_scaled_tbl_rownum(void)
{
#define ALLOC_CHUNK_SIZE 2
    unsigned int ncpus = sw_max_num_cpus;
    unsigned char *newtbl, *tmptbl;
    int retval = -1;

    // If we have space in the existing table, return that.
    pw_pr_debug("   stgitr: Entry: alloced: %d, avail: %d",
                sw_telem_rows_alloced, sw_telem_rows_avail);
    if (sw_telem_rows_avail) {
        retval = sw_telem_rows_alloced - sw_telem_rows_avail;
        sw_telem_rows_avail--;
        return retval;
    }

    /* Alocate a new table to hold both new and old values. */
    newtbl = sw_kmalloc(ncpus * (sw_telem_rows_alloced + ALLOC_CHUNK_SIZE),
                        GFP_KERNEL);
    if (newtbl == NULL){
        return -1;
    }

    /* Copy existing IDs into new storage. */
    if (sw_telem_scaled_ids != NULL) {
        memmove(newtbl, sw_telem_scaled_ids, ncpus * sw_telem_rows_alloced);
    }

    /* Initialize the new table entries. */
    memset(&newtbl[ncpus * sw_telem_rows_alloced],0,(ncpus * ALLOC_CHUNK_SIZE));

    /* Move the new mem into place, free the old mem, and update counters. */
    tmptbl = sw_telem_scaled_ids;
    sw_telem_scaled_ids = newtbl;
    if (tmptbl) {
        sw_kfree(tmptbl);
    }
    retval = sw_telem_rows_avail;
    sw_telem_rows_alloced += ALLOC_CHUNK_SIZE;
    sw_telem_rows_avail   = ALLOC_CHUNK_SIZE - 1;
    return retval;
}

/**
 * sw_free_telem_scaled_id_table - Free the allocated slots.
 * Returns: Nothing
 *
 * Admittedly, a more symmetrical function name would be nice.
 */
static void sw_telem_release_scaled_ids(void)
{
    sw_telem_rows_alloced = 0;
    sw_telem_rows_avail   = 0;
    if (sw_telem_scaled_ids) {
        sw_kfree(sw_telem_scaled_ids);
    }
    sw_telem_scaled_ids = NULL;
}

/**
 * sw_telem_scale_ids - Scale the IDs across the CPU range storing
 *                      the telemetry IDs in scaled_ids[].
 * @td:            Telemetry descriptor of id data
 * @ind_table_row: Table row to scale the IDs into.
 * Returns: PW_SUCCESS on success, -PW_ERROR on failure.
 *
 * Generate the IDs and populate a scaled ID table row by expanding
 * the expression stored in the telemetry descriptor.
 * Note that td->scale_val's of 0 are filtered out by the client.
 */
static int sw_telem_scale_ids(struct sw_driver_telem_io_descriptor *td,
                              int ind_table_row)
{
    unsigned int c, local_id;
    u8  unit    = td->unit;  /* Telemetry unit to use. */
    u16 base_id = td->id;    /* Base ID value */
    int idx;  /* Index into telemetry data array of event ID to gather. */
    u8 *scaled_ids;             /* Array of scaled ids */

    /* Get space for Allocate the space for the storage */
    scaled_ids = sw_get_instance_row_addr(ind_table_row);
    if (scaled_ids == NULL) {
        pw_pr_error("ERROR: Unable to allocate scaled ID row\n");
        return -PW_ERROR;
    }

    /* Now expand the expression into the scaled ID array. */
    for (c = 0; c < sw_max_num_cpus; c++) {
        switch (td->scale_op) {
        case TELEM_OP_ADD:  local_id = base_id + (c + td->scale_val); break;
        case TELEM_OP_MULT: local_id = base_id + (c * td->scale_val); break;
        case TELEM_OP_DIV:  local_id = base_id + (c / td->scale_val); break;
        case TELEM_OP_MOD:  local_id = base_id + (c % td->scale_val); break;
        default:
            pw_pr_error("ERROR: Telem ID scaling failed: "
                        "Unknown telem_op: 0x%x\n", td->scale_op);
            return -PW_ERROR;
            break;
        }

        /* Find the index of the requested event ID in the telemetry table,
         * and store that index in the instance table.
         */
        if ((idx = sw_telem_register_id(unit, local_id)) == -PW_ERROR) {
            return -PW_ERROR;
        }
        scaled_ids[c] = (u16)idx;
        pw_pr_debug("   tel_init: ID:%x &scaled_id[%d] = %d\n",
                    base_id, c, idx);
    }
    return PW_SUCCESS;
}

/**
 * sw_telem_init_func - Set up the telemetry unit to retrieve a data item
 *                        (e.g. counter).
 * @descriptor:  The IO descriptor containing the unit and ID
 *                        of the telemetry info to gather.
 *
 * Because we don't (currently) control all of the counters, we
 * economize by seeing if it's already being collected before allocate
 * a slot for it.
 *
 * Returns: PW_SUCCESS  if the telem collector can collect the requested data.
 *         -PW_ERROR   if the the addition of that item fails.
 */
int sw_telem_init_func(struct sw_driver_io_descriptor *descriptor)
{
    struct sw_driver_telem_io_descriptor *td = &(descriptor->telem_descriptor);
    u8  unit = td->unit;  /* Telemetry unit to use. */
    u32 id; /* Event ID we want telemetry to track. */
    int idx;  /* Index into telemetry data array of event ID to gather. */

    if (!telemetry_available()) {
        return -PW_ERROR;
    }

    id = (u32)(td->id);
    pw_pr_debug("sw_telem_init_func(): requested metric: unit=%d id=0x%x\n",
                unit, id);

    /* Turn the sampling period down, so that we can update the unit. */
    telemetry_set_sampling_period((unit==TELEM_PUNIT) ? TELEM_SAMPLING_1S : 0,
                                  (unit==TELEM_PMC)   ? TELEM_SAMPLING_1S : 0);

    /* Is this ID identical across modules/cpus? */
    if (IS_SCALED_ID(td)) {
        int scaled_idx;   /* Row number of the scaled ID table */

        pw_pr_debug("----tel_INIT: DESC = %p, td: %p &td->id=%p is_instanced: ID=0x%x\n",
                    descriptor, td, &(td->id), td->id);

        /* Fetch an Instance ID table row number */
        if ((scaled_idx = sw_telem_get_scaled_tbl_rownum()) == -1) {
            sw_reset_telem(descriptor);
            return -PW_ERROR;
        }

        /* Expand the IDs into a scaled_ID table row. */
        if (sw_telem_scale_ids(td, scaled_idx) == -PW_ERROR) {
            sw_reset_telem(descriptor);
            return -PW_ERROR;
        }
        /* Finally, save scaled table row number in descriptor's 'idx' */
        td->idx = scaled_idx;

    } else {
        pw_pr_debug("   tel_init: is NOT instanced: ID=0x%x", td->id);
        /* Not instanced, so just register it as-is. */
        if ((idx = sw_telem_register_id(unit, id)) == -PW_ERROR) {
            return -PW_ERROR;
        }
        /* Invariant: idx contains the index of the new data item. */
        /* Save the index for later fast lookup. */
        td->idx = (u16)idx;
        pw_pr_debug("sw_telem_init_func(): metric: unit=%d id=0x%x idx=%x\n",
                    unit, id, idx);
    }

    // Now set the sampling period to around 1 MS
    telemetry_set_sampling_period((unit==TELEM_PUNIT) ? TELEM_SAMPLING_1MS : 0,
                                  (unit==TELEM_PMC)   ? TELEM_SAMPLING_1MS : 0);

    return PW_SUCCESS;
}


/**
 * sw_read_telem_info - Read a metric's data from the telemetry driver.
 * @dest:               Destination (storage for the read data)
 * @cpu:                Which CPU to read from (not used)
 * @descriptor:         The descriptor containing the data ID to read
 * @data_size_in_bytes: The # of bytes in the result (always 8)
 *
 * Returns: Nothing, but stores SW_TELEM_READ_FAIL_VALUE to dest if the read fails.
 */
void sw_read_telem_info(char *dest, int cpu,
                          const sw_driver_io_descriptor_t *descriptor,
                          u16 data_size_in_bytes)
{
    int  len;
    u64 *data_dest = (u64 *)dest;
    int retry_count;
    const struct sw_driver_telem_io_descriptor *td = &(descriptor->telem_descriptor);
    unsigned int idx;

#define TELEM_PKT_SIZE 16  /* sizeof(struct telemetry_evtlog) + padding */
    struct telemetry_evtlog events[MAX_TELEM_EVENTS];

    if (!telemetry_available()) {
        return;
    }

    // Get the event index
    if (IS_SCALED_ID(td)) {
        unsigned char *scaled_ids;
        scaled_ids = sw_get_instance_row_addr(td->idx);
        if (scaled_ids == NULL) {
            pw_pr_error("Sw_read_telem_info_i: Illegal row index: *%p = %d",
                        &td->idx, td->idx);
            *data_dest = SW_TELEM_READ_FAIL_VALUE;
            return;  /* Don't set the dest/data buffer. */
        }
        idx = scaled_ids[RAW_CPU()]; /* Get per-cpu entry */
    } else {
        idx = td->idx;
    }

    /*
     * Because of the enormous overhead of reading telemetry data from
     * the current kernel driver, failure to read the data is not
     * unheard of.  As such, 3 times, should the read fail.  Once we
     * get a higher-performance read routine, we should be able to
     * eliminate this retry (or maybe decrease it.)
     */
    retry_count = 3;
    while (retry_count--) {
        len = telemetry_raw_read_eventlog(descriptor->telem_descriptor.unit,
                                          events,
                                          sizeof(events) / TELEM_PKT_SIZE);

        if ((len < 0) || (len < idx)) {
            pw_pr_debug("sw_read_telem_info_i: read failed: len=%d\n", len);
            if (retry_count == 0) {
                *data_dest = SW_TELEM_READ_FAIL_VALUE;
                return;  /* Don't set the dest/data buffer. */
            }
        } else {
            break;
        }
    }
    // TODO: Resolve if we should return something other than
    //       SW_TELEM_READ_FAIL_VALUE, if the actual data happens to be that.
    *data_dest = events[idx].telem_evtlog;
#ifdef TOO_LOUD
    /* Very noisy debugging code which reports every read of a changed value.
     * Only works when you are reading exactly one telemetry value.
     */
    {
        static u64 counter;
        static u64 lastval = ~0;
        counter  = events[idx].telem_evtlog;
        if (counter != lastval) {
            /* r = read operation, s = slot, i = ID, d = data value. */
            printk(KERN_INFO "r s %d i %x d 0x%llx\n",
                        idx, events[idx].telem_evtid, events[idx].telem_evtlog);
            lastval = counter;
        }
    }
#endif
}

/**
 * sw_reset_telem - Stop collecting telemetry info.
 * @descriptor: Unused in this function
 *
 * Stop collecting anything extra, and give the driver back to
 * debugfs.  Because this driver increases the sampling rate, the
 * kernel's telemetry driver can't succesfully reset the driver unless
 * we first drop the rate back down to a much slower rate.  This is a
 * temporary measure, since the reset operation will then reset the
 * sampling interval to whatever the GMIN driver wants.
 *
 * Return: PW_SUCCESS.
 */
int sw_reset_telem(const struct sw_driver_io_descriptor *descriptor)
{
    if (telemetry_available()) {
        telemetry_set_sampling_period(TELEM_SAMPLING_1S,
                                      TELEM_SAMPLING_1S);
        telemetry_reset_events();
        sw_telem_release_scaled_ids();
    }
    return PW_SUCCESS;
}

/**
 * sw_available_telem -- Decide if the telemetry subsystem is available for use
 */
bool sw_telem_available(void)
{
    return telemetry_available();
};
