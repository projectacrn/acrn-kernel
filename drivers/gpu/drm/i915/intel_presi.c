#include "i915_drv.h"
#include "i915_params.h"

static const char * const presi_mode_names[] = {
	[I915_PRESI_MODE_UNKNOWN] = "unknown",
	[I915_PRESI_MODE_NONE] = "none (silicon)",
	[I915_PRESI_MODE_SIMULATOR] = "simulation",
	[I915_PRESI_MODE_EMULATOR_PIPEGT] = "emulation pipeGT",
	[I915_PRESI_MODE_EMULATOR_PIPE2D] = "emulation pipe2D",
};

#define INTEL_PCH_HAS4_DEVICE_ID_TYPE  0x3a00 /* simics + HAS */
static inline bool is_presi_pch(unsigned short id)
{
	return id == INTEL_PCH_HAS4_DEVICE_ID_TYPE;
}

/**
 * intel_presi_need_virt_pch - returns whether we're running pre-si and
 * virt pch detection is required
 * @i915:	i915 device
 *
 * Return: true if virt pch detection is required, false otherwise
 */
bool intel_presi_need_virt_pch(struct drm_i915_private *i915, unsigned short id)
{
	return i915->presi_info.mode >= I915_PRESI_MODE_SIMULATOR ||
		is_presi_pch(id);
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
	BUILD_BUG_ON(I915_PRESI_MODE_AUTODETECT >= 0);
	GEM_BUG_ON(i915->presi_info.mode != I915_PRESI_MODE_UNKNOWN);

	if (mode >= I915_PRESI_MODE_NONE && mode <= I915_MAX_PRESI_MODE) {
		DRM_DEBUG_DRIVER("using pre-silicon mode from modparam: %s\n",
				 presi_mode_names[mode]);
		i915->presi_info.mode = mode;
	} else if (mode != I915_PRESI_MODE_AUTODETECT) {
		DRM_ERROR("invalid pre-silicon mode %d selected in "
			  "modparam! defaulting to silicon mode\n",
			  i915_modparams.presi_mode);
		i915->presi_info.mode = I915_PRESI_MODE_NONE;
	}

	return;
}

/**
 * intel_detect_presi_env - check if we're running on a pre-si environemnt
 * @i915:	i915 device
 * @id:		PCH id
 *
 * This function only autodetects if we're running in pre-silicon or not based
 * on the PCH id, but doesn't detect the pre-silicon mode (sim or emu). Any
 * check to IS_SIMULATION and similar will trigger a GEM_BUG_ON until the mode
 * detection is complete in intel_detect_presi_mode
 */
void intel_detect_presi_env(struct drm_i915_private *i915, unsigned short id)
{
	if (i915->presi_info.mode > 0)
		return;

	/* we use I915_PRESI_MODE_AUTODETECT mode here to mark that we're in
	 * pre-si, we'll detect the correct environemnt after the mmio init.
	 * Checking for simulation vs emulation will BUG() until we mark the
	 * detection as done, so no risk of false matching
	 */

	if (is_presi_pch(id)) {
		DRM_DEBUG_DRIVER("Found HAS4 PCH (x58 ICH10)\n");
		i915->presi_info.mode = I915_PRESI_MODE_AUTODETECT;
	}
}

#define HAS_IS_ACTIVE BIT(31)
#define EMU_IS_ACTIVE BIT(29)
#define HAS_STATUS_VALID(status) (((status) != ~0U) && ((status) & HAS_IS_ACTIVE))
/*
 * the lower 4 bits of the status indicate HAS_MODE. BIT(3), which indicates
 * the VM mode used in HAS3+, is always set for our usecases, so we can ignore
 * it. The lower 3 bits can be set to a value via the simics start script,
 * following a table of recommended setting. The only one we're currently
 * interested in is HASMODE_PIPE2D (5)
 */
#define HAS_MODE_MASK 0x7
#define HAS_MODE_PIPE2D 5
#define HAS_IS_PIPE2D(status) (((status) & HAS_MODE_MASK) == HAS_MODE_PIPE2D)
static u32 get_has_status(struct drm_i915_private *i915)
{
	u32 read_value = 0;
	const i915_reg_t has_status_reg = _MMIO(0x180008);

	read_value = __raw_i915_read32(i915, has_status_reg);

	/* Temporary fallback: HAS status MMIO register was checked into HAS
	 * mid-december 2017 so only releases from then onwards have it. We keep
	 * a fallback to the old way of getting the status (pci cfg) to give
	 * time to everyone to move to a supported simics version. Note that
	 * when the driver is running in a virtual machine we can only use the
	 * MMIO interface (because the hypervisor intercepts the pci accesses),
	 * so that's the preferred one long term.
	 *
	 * TODO: remove below fallback before any virtualization code (gvt or
	 * SRIOV) is enabled pre-silicon
	 */
	if (!HAS_STATUS_VALID(read_value)) {
		WARN_ON(pci_write_config_dword(i915->drm.pdev,
					       0xF8, /* CFG_HAS_COMMAND_OFFSET */
					       0x10)); /* CFG_HAS_STATUS */

		WARN_ON(pci_read_config_dword(i915->drm.pdev,
					      0xFC, /* CFG_HAS_DATA_OFFSET */
					      &read_value));
	}

	return read_value;
}

/**
 * intel_detect_presi_mode - autodetect the pre-si mode (sim or emu)
 * @i915:	i915 device
 *
 * This function autodetects the pre-silicon mode (sim or emu) based on the
 * information provided by HAS.
 */
void intel_detect_presi_mode(struct drm_i915_private *i915)
{
	u32 has_status;

	if (i915->presi_info.mode > 0)
		return;

	/*
	 * if we're in pre-silicon we expect this function to be called after
	 * intel_detect_presi has written I915_PRESI_MODE_AUTODETECT to
	 * presi_info.mode, which means that if the value is different we've
	 * not running in presilicon and we can skip reading status from HAS.
	 */
	if (i915->presi_info.mode != I915_PRESI_MODE_AUTODETECT) {
		i915->presi_info.mode = I915_PRESI_MODE_NONE;
		return;
	}

	has_status = get_has_status(i915);

	if (!HAS_STATUS_VALID(has_status)) {
		DRM_ERROR("invalid HAS status 0x%x! selecting simulator"
			  " mode\n", has_status);
		i915->presi_info.mode = I915_PRESI_MODE_SIMULATOR;
	} else if (has_status & EMU_IS_ACTIVE) {
		if (HAS_IS_PIPE2D(has_status))
			i915->presi_info.mode = I915_PRESI_MODE_EMULATOR_PIPE2D;
		else
			i915->presi_info.mode = I915_PRESI_MODE_EMULATOR_PIPEGT;
	} else {
		i915->presi_info.mode = I915_PRESI_MODE_SIMULATOR;
	}

	DRM_DEBUG_DRIVER("auto-detected pre-si mode: %s\n",
			 presi_mode_names[i915->presi_info.mode]);
}
