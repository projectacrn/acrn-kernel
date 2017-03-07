#ifndef __SW_TRACE_NOTIFIER_PROVIDER_H__
#define __SW_TRACE_NOTIFIER_PROVIDER_H__

/*
 * Some architectures and OS versions require a "discovery"
 * phase for tracepoints and/or notifiers. Allow for that here.
 */
int sw_extract_trace_notifier_providers(void);
/*
 * Reset trace/notifier providers at the end
 * of a collection.
 */
void sw_reset_trace_notifier_providers(void);
/*
 * Print statistics on trace/notifier provider overheads.
 */
void sw_print_trace_notifier_provider_overheads(void);
/*
 * Add all trace/notifier providers.
 */
int sw_add_trace_notifier_providers(void);
/*
 * Remove previously added providers.
 */
void sw_remove_trace_notifier_providers(void);
#endif // __SW_TRACE_NOTIFIER_PROVIDER_H__
