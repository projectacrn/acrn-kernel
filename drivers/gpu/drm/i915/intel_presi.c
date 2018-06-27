#include "i915_drv.h"
#include "i915_params.h"

static const char * const presi_mode_names[] = {
	[I915_PRESI_MODE_UNKNOWN] = "unknown",
	[I915_PRESI_MODE_NONE] = "none (silicon)",
	[I915_PRESI_MODE_SIMULATOR] = "simulation",
	[I915_PRESI_MODE_EMULATOR_PIPEGT] = "emulation pipeGT",
	[I915_PRESI_MODE_EMULATOR_PIPE2D] = "emulation pipe2D",
};

/**
 * intel_presi_need_virt_pch - returns whether we're running pre-si and
 * virt pch detection is required
 * @i915:	i915 device
 *
 * Return: true if virt pch detection is required, false otherwise
 */
bool intel_presi_need_virt_pch(struct drm_i915_private *i915)
{
	return i915->presi_info.mode >= I915_PRESI_MODE_SIMULATOR;
}

/**
 * intel_presi_init - checks the pre-si modparam and acts on it
 * @i915:	i915 device
 *
 * presi_mode is only updated if the modparam is set to a valid value. An
 * error is logged if the modparam is set incorrectly
 */
void intel_presi_init(struct drm_i915_private *i915)
{
	int mode = MODPARAM_TO_PRESI_MODE(i915_modparams.presi_mode);

	BUILD_BUG_ON(I915_PRESI_MODE_UNKNOWN); /* unknown needs to be 0 */
	GEM_BUG_ON(i915->presi_info.mode != I915_PRESI_MODE_UNKNOWN);

	if (mode >= I915_PRESI_MODE_NONE && mode <= I915_MAX_PRESI_MODE) {
		DRM_DEBUG_DRIVER("using pre-silicon mode from modparam: %s\n",
				 presi_mode_names[mode]);
		i915->presi_info.mode = mode;
	} else {
		DRM_ERROR("invalid pre-silicon mode %d selected in "
			  "modparam! defaulting to silicon mode\n",
			  i915_modparams.presi_mode);
		i915->presi_info.mode = I915_PRESI_MODE_NONE;
	}

	return;
}
