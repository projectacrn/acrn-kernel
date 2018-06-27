
#ifndef _INTEL_PRESI_H_
#define _INTEL_PRESI_H_

struct drm_i915_private;

/*
 * We support different pre-silicon modes:
 * - simulation: GPU is simulated. Model is functionally accurate but
 * 		 implementation does not necessarily match HW.
 * - emulation pipeGT: GT RTL is booted on FPGA, while the rest of the HW
 * 		       is simulated.
 * - emulation pipe2D: Display and Gunit RTL is booted on FPGA, while the rest
 * 		       of the HW is simulated.
 *
 * Note: the enum values for detected envs are equal to the modparam values + 1
 */
struct intel_presi_info {
	enum {
		I915_PRESI_MODE_UNKNOWN = 0, /* aka not detected yet */
		I915_PRESI_MODE_NONE = 1, /* aka SILICON */
		I915_PRESI_MODE_SIMULATOR = 2,
		I915_PRESI_MODE_EMULATOR_PIPEGT = 3,
		I915_PRESI_MODE_EMULATOR_PIPE2D = 4,
		I915_MAX_PRESI_MODE = I915_PRESI_MODE_EMULATOR_PIPE2D
	} mode;
};
#define MODPARAM_TO_PRESI_MODE(x) ((x) + 1)

#define IS_PRESI_MODE(i915, x) ({ \
	GEM_BUG_ON((i915)->presi_info.mode == I915_PRESI_MODE_UNKNOWN); \
	(i915)->presi_info.mode == I915_PRESI_MODE_##x; \
})

#define IS_PRESILICON(i915) (!IS_PRESI_MODE(i915, NONE))
#define IS_SIMULATOR(i915) (IS_PRESI_MODE(i915, SIMULATOR))
#define IS_PIPEGT_EMULATOR(i915) (IS_PRESI_MODE(i915, EMULATOR_PIPEGT))
#define IS_PIPE2D_EMULATOR(i915) (IS_PRESI_MODE(i915, EMULATOR_PIPE2D))
#define IS_EMULATOR(i915) (IS_PIPEGT_EMULATOR(i915) || IS_PIPE2D_EMULATOR(i915))

bool intel_presi_need_virt_pch(struct drm_i915_private *i915);
void intel_presi_init(struct drm_i915_private *i915);

#endif
