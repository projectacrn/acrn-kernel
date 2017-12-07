#
# Makefile for the socwatch driver.
#

ccflags-y += -Idrivers/platform/x86/socwatch/inc/

obj-$(CONFIG_INTEL_SOCWATCH)    += socwatch.o

socwatch-objs := sw_driver.o sw_hardware_io.o \
                 sw_output_buffer.o sw_tracepoint_handlers.o \
                 sw_mem.o sw_collector.o sw_telem.o \
                 sw_file_ops.o sw_internal.o sw_ops_provider.o \
                 sw_reader.o sw_trace_notifier_provider.o
