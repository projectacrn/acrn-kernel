#
# Makefile for the soc_perf driver.
#

ccflags-y += -Idrivers/platform/x86/socperf/inc/

# Enable EMON by default for non ia64 systems
# TODO: Add option for enabling
ifneq ($(ARCH), ia64)
	ccflagys-y += -DEMON -DEMON_INTERNAL

	arch-objs :=    pci.o                 \
			soc_uncore.o                  \
			haswellunc_sa.o               \
			npk_uncore.o
endif

obj-$(CONFIG_INTEL_SOCPERF)    += socperf.o

socperf-objs :=            \
		socperfdrv.o              \
		control.o                 \
		utility.o                 \
		$(arch-objs)

