/*
 * Copyright © 2016 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include "i915_drv.h"

#define PLATFORM_NAME(x) [INTEL_##x] = #x
static const char * const platform_names[] = {
	PLATFORM_NAME(I830),
	PLATFORM_NAME(I845G),
	PLATFORM_NAME(I85X),
	PLATFORM_NAME(I865G),
	PLATFORM_NAME(I915G),
	PLATFORM_NAME(I915GM),
	PLATFORM_NAME(I945G),
	PLATFORM_NAME(I945GM),
	PLATFORM_NAME(G33),
	PLATFORM_NAME(PINEVIEW),
	PLATFORM_NAME(I965G),
	PLATFORM_NAME(I965GM),
	PLATFORM_NAME(G45),
	PLATFORM_NAME(GM45),
	PLATFORM_NAME(IRONLAKE),
	PLATFORM_NAME(SANDYBRIDGE),
	PLATFORM_NAME(IVYBRIDGE),
	PLATFORM_NAME(VALLEYVIEW),
	PLATFORM_NAME(HASWELL),
	PLATFORM_NAME(BROADWELL),
	PLATFORM_NAME(CHERRYVIEW),
	PLATFORM_NAME(SKYLAKE),
	PLATFORM_NAME(BROXTON),
	PLATFORM_NAME(KABYLAKE),
	PLATFORM_NAME(GEMINILAKE),
	PLATFORM_NAME(COFFEELAKE),
	PLATFORM_NAME(CANNONLAKE),
	PLATFORM_NAME(ICELAKE),
};
#undef PLATFORM_NAME

const char *intel_platform_name(enum intel_platform platform)
{
	BUILD_BUG_ON(ARRAY_SIZE(platform_names) != INTEL_MAX_PLATFORMS);

	if (WARN_ON_ONCE(platform >= ARRAY_SIZE(platform_names) ||
			 platform_names[platform] == NULL))
		return "<unknown>";

	return platform_names[platform];
}

void intel_device_info_dump(struct drm_i915_private *dev_priv)
{
	const struct intel_device_info *info = &dev_priv->info;

	DRM_DEBUG_DRIVER("i915 device info: platform=%s gen=%i pciid=0x%04x rev=0x%02x",
			 intel_platform_name(info->platform),
			 info->gen,
			 dev_priv->drm.pdev->device,
			 dev_priv->drm.pdev->revision);
#define PRINT_FLAG(name) \
	DRM_DEBUG_DRIVER("i915 device info: " #name ": %s", yesno(info->name))
	DEV_INFO_FOR_EACH_FLAG(PRINT_FLAG);
#undef PRINT_FLAG
}

static void gen11_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct sseu_dev_info *sseu = &mkwrite_device_info(dev_priv)->sseu;
	int eu_max = 8;
	u32 eu_disable;

	sseu->slice_mask = I915_READ(GEN11_GT_SLICE_ENABLE) &
				GEN11_GT_S_ENA_MASK;
	sseu->subslice_mask = ~(I915_READ(GEN11_GT_SUBSLICE_DISABLE) &
				GEN11_GT_SS_DIS_MASK);
	eu_disable = I915_READ(GEN11_EU_DISABLE) & GEN11_GT_S_ENA_MASK;

	sseu->eu_per_subslice = eu_max - hweight32(eu_disable);
	sseu->eu_total = sseu->eu_per_subslice * hweight32(sseu->subslice_mask);

	/* ICL has no power gating restrictions. */
	sseu->has_slice_pg = 1;
	sseu->has_subslice_pg = 1;
	sseu->has_eu_pg = 1;
}

static void gen10_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct sseu_dev_info *sseu = &mkwrite_device_info(dev_priv)->sseu;
	const u32 fuse2 = I915_READ(GEN8_FUSE2);

	sseu->slice_mask = (fuse2 & GEN10_F2_S_ENA_MASK) >>
			    GEN10_F2_S_ENA_SHIFT;
	sseu->subslice_mask = (1 << 4) - 1;
	sseu->subslice_mask &= ~((fuse2 & GEN10_F2_SS_DIS_MASK) >>
				 GEN10_F2_SS_DIS_SHIFT);

	sseu->eu_total = hweight32(~I915_READ(GEN8_EU_DISABLE0));
	sseu->eu_total += hweight32(~I915_READ(GEN8_EU_DISABLE1));
	sseu->eu_total += hweight32(~I915_READ(GEN8_EU_DISABLE2));
	sseu->eu_total += hweight8(~(I915_READ(GEN10_EU_DISABLE3) &
				     GEN10_EU_DIS_SS_MASK));

	/*
	 * CNL is expected to always have a uniform distribution
	 * of EU across subslices with the exception that any one
	 * EU in any one subslice may be fused off for die
	 * recovery.
	 */
	sseu->eu_per_subslice = sseu_subslice_total(sseu) ?
				DIV_ROUND_UP(sseu->eu_total,
					     sseu_subslice_total(sseu)) : 0;

	/* No restrictions on Power Gating */
	sseu->has_slice_pg = 1;
	sseu->has_subslice_pg = 1;
	sseu->has_eu_pg = 1;
}

static void cherryview_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct sseu_dev_info *sseu = &mkwrite_device_info(dev_priv)->sseu;
	u32 fuse, eu_dis;

	fuse = I915_READ(CHV_FUSE_GT);

	sseu->slice_mask = BIT(0);

	if (!(fuse & CHV_FGT_DISABLE_SS0)) {
		sseu->subslice_mask |= BIT(0);
		eu_dis = fuse & (CHV_FGT_EU_DIS_SS0_R0_MASK |
				 CHV_FGT_EU_DIS_SS0_R1_MASK);
		sseu->eu_total += 8 - hweight32(eu_dis);
	}

	if (!(fuse & CHV_FGT_DISABLE_SS1)) {
		sseu->subslice_mask |= BIT(1);
		eu_dis = fuse & (CHV_FGT_EU_DIS_SS1_R0_MASK |
				 CHV_FGT_EU_DIS_SS1_R1_MASK);
		sseu->eu_total += 8 - hweight32(eu_dis);
	}

	/*
	 * CHV expected to always have a uniform distribution of EU
	 * across subslices.
	*/
	sseu->eu_per_subslice = sseu_subslice_total(sseu) ?
				sseu->eu_total / sseu_subslice_total(sseu) :
				0;
	/*
	 * CHV supports subslice power gating on devices with more than
	 * one subslice, and supports EU power gating on devices with
	 * more than one EU pair per subslice.
	*/
	sseu->has_slice_pg = 0;
	sseu->has_subslice_pg = sseu_subslice_total(sseu) > 1;
	sseu->has_eu_pg = (sseu->eu_per_subslice > 2);
}

static void gen9_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct intel_device_info *info = mkwrite_device_info(dev_priv);
	struct sseu_dev_info *sseu = &info->sseu;
	int s_max = 3, ss_max = 4, eu_max = 8;
	int s, ss;
	u32 fuse2, eu_disable;
	u8 eu_mask = 0xff;

	fuse2 = I915_READ(GEN8_FUSE2);
	sseu->slice_mask = (fuse2 & GEN8_F2_S_ENA_MASK) >> GEN8_F2_S_ENA_SHIFT;

	/*
	 * The subslice disable field is global, i.e. it applies
	 * to each of the enabled slices.
	*/
	sseu->subslice_mask = (1 << ss_max) - 1;
	sseu->subslice_mask &= ~((fuse2 & GEN9_F2_SS_DIS_MASK) >>
				 GEN9_F2_SS_DIS_SHIFT);

	/*
	 * Iterate through enabled slices and subslices to
	 * count the total enabled EU.
	*/
	for (s = 0; s < s_max; s++) {
		if (!(sseu->slice_mask & BIT(s)))
			/* skip disabled slice */
			continue;

		eu_disable = I915_READ(GEN9_EU_DISABLE(s));
		for (ss = 0; ss < ss_max; ss++) {
			int eu_per_ss;

			if (!(sseu->subslice_mask & BIT(ss)))
				/* skip disabled subslice */
				continue;

			eu_per_ss = eu_max - hweight8((eu_disable >> (ss*8)) &
						      eu_mask);

			/*
			 * Record which subslice(s) has(have) 7 EUs. we
			 * can tune the hash used to spread work among
			 * subslices if they are unbalanced.
			 */
			if (eu_per_ss == 7)
				sseu->subslice_7eu[s] |= BIT(ss);

			sseu->eu_total += eu_per_ss;
		}
	}

	/*
	 * SKL is expected to always have a uniform distribution
	 * of EU across subslices with the exception that any one
	 * EU in any one subslice may be fused off for die
	 * recovery. BXT is expected to be perfectly uniform in EU
	 * distribution.
	*/
	sseu->eu_per_subslice = sseu_subslice_total(sseu) ?
				DIV_ROUND_UP(sseu->eu_total,
					     sseu_subslice_total(sseu)) : 0;
	/*
	 * SKL+ supports slice power gating on devices with more than
	 * one slice, and supports EU power gating on devices with
	 * more than one EU pair per subslice. BXT+ supports subslice
	 * power gating on devices with more than one subslice, and
	 * supports EU power gating on devices with more than one EU
	 * pair per subslice.
	*/
	sseu->has_slice_pg =
		!IS_GEN9_LP(dev_priv) && hweight8(sseu->slice_mask) > 1;
	sseu->has_subslice_pg =
		IS_GEN9_LP(dev_priv) && sseu_subslice_total(sseu) > 1;
	sseu->has_eu_pg = sseu->eu_per_subslice > 2;

	if (IS_GEN9_LP(dev_priv)) {
#define IS_SS_DISABLED(ss)	(!(sseu->subslice_mask & BIT(ss)))
		info->has_pooled_eu = hweight8(sseu->subslice_mask) == 3;

		sseu->min_eu_in_pool = 0;
		if (info->has_pooled_eu) {
			if (IS_SS_DISABLED(2) || IS_SS_DISABLED(0))
				sseu->min_eu_in_pool = 3;
			else if (IS_SS_DISABLED(1))
				sseu->min_eu_in_pool = 6;
			else
				sseu->min_eu_in_pool = 9;
		}
#undef IS_SS_DISABLED
	}
}

static void broadwell_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct sseu_dev_info *sseu = &mkwrite_device_info(dev_priv)->sseu;
	const int s_max = 3, ss_max = 3, eu_max = 8;
	int s, ss;
	u32 fuse2, eu_disable[3]; /* s_max */

	fuse2 = I915_READ(GEN8_FUSE2);
	sseu->slice_mask = (fuse2 & GEN8_F2_S_ENA_MASK) >> GEN8_F2_S_ENA_SHIFT;
	/*
	 * The subslice disable field is global, i.e. it applies
	 * to each of the enabled slices.
	 */
	sseu->subslice_mask = GENMASK(ss_max - 1, 0);
	sseu->subslice_mask &= ~((fuse2 & GEN8_F2_SS_DIS_MASK) >>
				 GEN8_F2_SS_DIS_SHIFT);

	eu_disable[0] = I915_READ(GEN8_EU_DISABLE0) & GEN8_EU_DIS0_S0_MASK;
	eu_disable[1] = (I915_READ(GEN8_EU_DISABLE0) >> GEN8_EU_DIS0_S1_SHIFT) |
			((I915_READ(GEN8_EU_DISABLE1) & GEN8_EU_DIS1_S1_MASK) <<
			 (32 - GEN8_EU_DIS0_S1_SHIFT));
	eu_disable[2] = (I915_READ(GEN8_EU_DISABLE1) >> GEN8_EU_DIS1_S2_SHIFT) |
			((I915_READ(GEN8_EU_DISABLE2) & GEN8_EU_DIS2_S2_MASK) <<
			 (32 - GEN8_EU_DIS1_S2_SHIFT));

	/*
	 * Iterate through enabled slices and subslices to
	 * count the total enabled EU.
	 */
	for (s = 0; s < s_max; s++) {
		if (!(sseu->slice_mask & BIT(s)))
			/* skip disabled slice */
			continue;

		for (ss = 0; ss < ss_max; ss++) {
			u32 n_disabled;

			if (!(sseu->subslice_mask & BIT(ss)))
				/* skip disabled subslice */
				continue;

			n_disabled = hweight8(eu_disable[s] >> (ss * eu_max));

			/*
			 * Record which subslices have 7 EUs.
			 */
			if (eu_max - n_disabled == 7)
				sseu->subslice_7eu[s] |= 1 << ss;

			sseu->eu_total += eu_max - n_disabled;
		}
	}

	/*
	 * BDW is expected to always have a uniform distribution of EU across
	 * subslices with the exception that any one EU in any one subslice may
	 * be fused off for die recovery.
	 */
	sseu->eu_per_subslice = sseu_subslice_total(sseu) ?
				DIV_ROUND_UP(sseu->eu_total,
					     sseu_subslice_total(sseu)) : 0;

	/*
	 * BDW supports slice power gating on devices with more than
	 * one slice.
	 */
	sseu->has_slice_pg = hweight8(sseu->slice_mask) > 1;
	sseu->has_subslice_pg = 0;
	sseu->has_eu_pg = 0;
}

static u32 read_reference_ts_freq(struct drm_i915_private *dev_priv)
{
	u32 ts_override = I915_READ(GEN9_TIMESTAMP_OVERRIDE);
	u32 base_freq, frac_freq;

	base_freq = ((ts_override & GEN9_TIMESTAMP_OVERRIDE_US_COUNTER_DIVIDER_MASK) >>
		     GEN9_TIMESTAMP_OVERRIDE_US_COUNTER_DIVIDER_SHIFT) + 1;
	base_freq *= 1000;

	frac_freq = ((ts_override &
		      GEN9_TIMESTAMP_OVERRIDE_US_COUNTER_DENOMINATOR_MASK) >>
		     GEN9_TIMESTAMP_OVERRIDE_US_COUNTER_DENOMINATOR_SHIFT);
	frac_freq = 1000 / (frac_freq + 1);

	return base_freq + frac_freq;
}

static u32 gen10_get_crystal_clock_freq(struct drm_i915_private *dev_priv,
					u32 rpm_config_reg)
{
	u32 f19_2_mhz = 19200;
	u32 f24_mhz = 24000;
	u32 crystal_clock = (rpm_config_reg &
			     GEN9_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_MASK) >>
			    GEN9_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_SHIFT;

	switch (crystal_clock) {
	case GEN9_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_19_2_MHZ:
		return f19_2_mhz;
	case GEN9_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_24_MHZ:
		return f24_mhz;
	default:
		MISSING_CASE(crystal_clock);
		return 0;
	}
}

static u32 gen11_get_crystal_clock_freq(struct drm_i915_private *dev_priv,
					u32 rpm_config_reg)
{
	u32 f19_2_mhz = 19200;
	u32 f24_mhz = 24000;
	u32 f25_mhz = 25000;
	u32 f38_4_mhz = 38400;
	u32 crystal_clock = (rpm_config_reg &
			     GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_MASK) >>
			    GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_SHIFT;

	switch (crystal_clock) {
	case GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_24_MHZ:
		return f24_mhz;
	case GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_19_2_MHZ:
		return f19_2_mhz;
	case GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_38_4_MHZ:
		return f38_4_mhz;
	case GEN11_RPM_CONFIG0_CRYSTAL_CLOCK_FREQ_25_MHZ:
		return f25_mhz;
	default:
		MISSING_CASE(crystal_clock);
		return 0;
	}
}

static u32 read_timestamp_frequency(struct drm_i915_private *dev_priv)
{
	u32 f12_5_mhz = 12500;
	u32 f19_2_mhz = 19200;
	u32 f24_mhz = 24000;

	if (INTEL_GEN(dev_priv) <= 4) {
		/* PRMs say:
		 *
		 *     "The value in this register increments once every 16
		 *      hclks." (through the “Clocking Configuration”
		 *      (“CLKCFG”) MCHBAR register)
		 */
		return dev_priv->rawclk_freq / 16;
	} else if (INTEL_GEN(dev_priv) <= 8) {
		/* PRMs say:
		 *
		 *     "The PCU TSC counts 10ns increments; this timestamp
		 *      reflects bits 38:3 of the TSC (i.e. 80ns granularity,
		 *      rolling over every 1.5 hours).
		 */
		return f12_5_mhz;
	} else if (INTEL_GEN(dev_priv) <= 9) {
		u32 ctc_reg = I915_READ(CTC_MODE);
		u32 freq = 0;

		if ((ctc_reg & CTC_SOURCE_PARAMETER_MASK) == CTC_SOURCE_DIVIDE_LOGIC) {
			freq = read_reference_ts_freq(dev_priv);
		} else {
			freq = IS_GEN9_LP(dev_priv) ? f19_2_mhz : f24_mhz;

			/* Now figure out how the command stream's timestamp
			 * register increments from this frequency (it might
			 * increment only every few clock cycle).
			 */
			freq >>= 3 - ((ctc_reg & CTC_SHIFT_PARAMETER_MASK) >>
				      CTC_SHIFT_PARAMETER_SHIFT);
		}

		return freq;
	} else if (INTEL_GEN(dev_priv) <= 11) {
		u32 ctc_reg = I915_READ(CTC_MODE);
		u32 freq = 0;

		/* First figure out the reference frequency. There are 2 ways
		 * we can compute the frequency, either through the
		 * TIMESTAMP_OVERRIDE register or through RPM_CONFIG. CTC_MODE
		 * tells us which one we should use.
		 */
		if ((ctc_reg & CTC_SOURCE_PARAMETER_MASK) == CTC_SOURCE_DIVIDE_LOGIC) {
			freq = read_reference_ts_freq(dev_priv);
		} else {
			u32 rpm_config_reg = I915_READ(RPM_CONFIG0);

			if (INTEL_GEN(dev_priv) <= 10)
				freq = gen10_get_crystal_clock_freq(dev_priv,
								rpm_config_reg);
			else
				freq = gen11_get_crystal_clock_freq(dev_priv,
								rpm_config_reg);

			/* Now figure out how the command stream's timestamp
			 * register increments from this frequency (it might
			 * increment only every few clock cycle).
			 */
			freq >>= 3 - ((rpm_config_reg &
				       GEN10_RPM_CONFIG0_CTC_SHIFT_PARAMETER_MASK) >>
				      GEN10_RPM_CONFIG0_CTC_SHIFT_PARAMETER_SHIFT);
		}

		return freq;
	}

	DRM_ERROR("Unknown gen, unable to compute command stream timestamp frequency\n");
	return 0;
}

/*
 * Determine various intel_device_info fields at runtime.
 *
 * Use it when either:
 *   - it's judged too laborious to fill n static structures with the limit
 *     when a simple if statement does the job,
 *   - run-time checks (eg read fuse/strap registers) are needed.
 *
 * This function needs to be called:
 *   - after the MMIO has been setup as we are reading registers,
 *   - after the PCH has been detected,
 *   - before the first usage of the fields it can tweak.
 */
void intel_device_info_runtime_init(struct drm_i915_private *dev_priv)
{
	struct intel_device_info *info = mkwrite_device_info(dev_priv);
	enum pipe pipe;

	if (INTEL_GEN(dev_priv) >= 10) {
		for_each_pipe(dev_priv, pipe)
			info->num_scalers[pipe] = 2;
	} else if (INTEL_GEN(dev_priv) == 9) {
		info->num_scalers[PIPE_A] = 2;
		info->num_scalers[PIPE_B] = 2;
		info->num_scalers[PIPE_C] = 1;
	}

	BUILD_BUG_ON(I915_NUM_ENGINES >
		     sizeof(intel_ring_mask_t) * BITS_PER_BYTE);

	/*
	 * Skylake and Broxton currently don't expose the topmost plane as its
	 * use is exclusive with the legacy cursor and we only want to expose
	 * one of those, not both. Until we can safely expose the topmost plane
	 * as a DRM_PLANE_TYPE_CURSOR with all the features exposed/supported,
	 * we don't expose the topmost plane at all to prevent ABI breakage
	 * down the line.
	 */
	if (IS_GEN10(dev_priv) || IS_GEMINILAKE(dev_priv))
		for_each_pipe(dev_priv, pipe)
			info->num_sprites[pipe] = 3;
	else if (IS_BROXTON(dev_priv)) {
		info->num_sprites[PIPE_A] = 2;
		info->num_sprites[PIPE_B] = 2;
		info->num_sprites[PIPE_C] = 1;
	} else if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		for_each_pipe(dev_priv, pipe)
			info->num_sprites[pipe] = 2;
	} else if (INTEL_GEN(dev_priv) >= 5 || IS_G4X(dev_priv)) {
		for_each_pipe(dev_priv, pipe)
			info->num_sprites[pipe] = 1;
	}

	if (i915_modparams.disable_display) {
		DRM_INFO("Display disabled (module parameter)\n");
		info->num_pipes = 0;
	} else if (info->num_pipes > 0 &&
		   (IS_GEN7(dev_priv) || IS_GEN8(dev_priv)) &&
		   HAS_PCH_SPLIT(dev_priv)) {
		u32 fuse_strap = I915_READ(FUSE_STRAP);
		u32 sfuse_strap = I915_READ(SFUSE_STRAP);

		/*
		 * SFUSE_STRAP is supposed to have a bit signalling the display
		 * is fused off. Unfortunately it seems that, at least in
		 * certain cases, fused off display means that PCH display
		 * reads don't land anywhere. In that case, we read 0s.
		 *
		 * On CPT/PPT, we can detect this case as SFUSE_STRAP_FUSE_LOCK
		 * should be set when taking over after the firmware.
		 */
		if (fuse_strap & ILK_INTERNAL_DISPLAY_DISABLE ||
		    sfuse_strap & SFUSE_STRAP_DISPLAY_DISABLED ||
		    (HAS_PCH_CPT(dev_priv) &&
		     !(sfuse_strap & SFUSE_STRAP_FUSE_LOCK))) {
			DRM_INFO("Display fused off, disabling\n");
			info->num_pipes = 0;
		} else if (fuse_strap & IVB_PIPE_C_DISABLE) {
			DRM_INFO("PipeC fused off\n");
			info->num_pipes -= 1;
		}
	} else if (info->num_pipes > 0 && IS_GEN9(dev_priv)) {
		u32 dfsm = I915_READ(SKL_DFSM);
		u8 disabled_mask = 0;
		bool invalid;
		int num_bits;

		if (dfsm & SKL_DFSM_PIPE_A_DISABLE)
			disabled_mask |= BIT(PIPE_A);
		if (dfsm & SKL_DFSM_PIPE_B_DISABLE)
			disabled_mask |= BIT(PIPE_B);
		if (dfsm & SKL_DFSM_PIPE_C_DISABLE)
			disabled_mask |= BIT(PIPE_C);

		num_bits = hweight8(disabled_mask);

		switch (disabled_mask) {
		case BIT(PIPE_A):
		case BIT(PIPE_B):
		case BIT(PIPE_A) | BIT(PIPE_B):
		case BIT(PIPE_A) | BIT(PIPE_C):
			invalid = true;
			break;
		default:
			invalid = false;
		}

		if (num_bits > info->num_pipes || invalid)
			DRM_ERROR("invalid pipe fuse configuration: 0x%x\n",
				  disabled_mask);
		else
			info->num_pipes -= num_bits;
	}

	/* Initialize slice/subslice/EU info */
	if (IS_CHERRYVIEW(dev_priv))
		cherryview_sseu_info_init(dev_priv);
	else if (IS_BROADWELL(dev_priv))
		broadwell_sseu_info_init(dev_priv);
	else if (INTEL_GEN(dev_priv) == 9)
		gen9_sseu_info_init(dev_priv);
	else if (INTEL_GEN(dev_priv) == 10)
		gen10_sseu_info_init(dev_priv);
	else if (INTEL_INFO(dev_priv)->gen >= 11)
		gen11_sseu_info_init(dev_priv);

	/* Initialize command stream timestamp frequency */
	info->cs_timestamp_frequency_khz = read_timestamp_frequency(dev_priv);

	DRM_DEBUG_DRIVER("slice mask: %04x\n", info->sseu.slice_mask);
	DRM_DEBUG_DRIVER("slice total: %u\n", hweight8(info->sseu.slice_mask));
	DRM_DEBUG_DRIVER("subslice total: %u\n",
			 sseu_subslice_total(&info->sseu));
	DRM_DEBUG_DRIVER("subslice mask %04x\n", info->sseu.subslice_mask);
	DRM_DEBUG_DRIVER("subslice per slice: %u\n",
			 hweight8(info->sseu.subslice_mask));
	DRM_DEBUG_DRIVER("EU total: %u\n", info->sseu.eu_total);
	DRM_DEBUG_DRIVER("EU per subslice: %u\n", info->sseu.eu_per_subslice);
	DRM_DEBUG_DRIVER("has slice power gating: %s\n",
			 info->sseu.has_slice_pg ? "y" : "n");
	DRM_DEBUG_DRIVER("has subslice power gating: %s\n",
			 info->sseu.has_subslice_pg ? "y" : "n");
	DRM_DEBUG_DRIVER("has EU power gating: %s\n",
			 info->sseu.has_eu_pg ? "y" : "n");
	DRM_DEBUG_DRIVER("CS timestamp frequency: %u kHz\n",
			 info->cs_timestamp_frequency_khz);
}

/*
 * Determine which engines are fused off in our particular hardware.
 *
 * This function needs to be called after the MMIO has been setup (as we need
 * to read registers) but before uncore init (because the powerwell for the
 * fused off engines doesn't exist, so we cannot initialize forcewake for them)
 */
void intel_device_info_fused_off_engines(struct drm_i915_private *dev_priv)
{
	struct intel_device_info *info = mkwrite_device_info(dev_priv);
	u32 media_fuse;
	uint logical_vdbox = 0;
	int i;

	if (INTEL_GEN(dev_priv) < 11)
		return;

	GEM_BUG_ON(!dev_priv->regs);

	media_fuse = I915_READ_FW(GEN11_GT_VEBOX_VDBOX_DISABLE);

	info->vdbox_disable = media_fuse & GEN11_GT_VDBOX_DISABLE_MASK;
	info->vebox_disable = (media_fuse & GEN11_GT_VEBOX_DISABLE_MASK) >>
				GEN11_GT_VEBOX_DISABLE_SHIFT;

	DRM_DEBUG_DRIVER("vdbox disable: %04x\n", info->vdbox_disable);
	for (i = 0; i < I915_MAX_VCS; i++) {
		if (!HAS_ENGINE(dev_priv, _VCS(i)))
			continue;

		if (!(BIT(i) & info->vdbox_disable)) {
			/*
			 * In Gen11, only even numbered logical VDBOXes are
			 * hooked up to an SFC (Scaler & Format Converter) unit.
			 */
			if (logical_vdbox++ % 2 == 0)
				info->vdbox_sfc_access |= BIT(i);
			continue;
		}

		info->ring_mask &= ~ENGINE_MASK(_VCS(i));
		WARN_ON(dev_priv->uncore.fw_domains &
				BIT(FW_DOMAIN_ID_MEDIA_VDBOX0 + i));
		DRM_DEBUG_DRIVER("vcs%u fused off\n", i);
	}

	DRM_DEBUG_DRIVER("vebox disable: %04x\n", info->vebox_disable);
	for (i = 0; i < I915_MAX_VECS; i++) {
		if (!HAS_ENGINE(dev_priv, _VECS(i)))
			continue;

		if (!(BIT(i) & info->vebox_disable))
			continue;

		info->ring_mask &= ~ENGINE_MASK(_VECS(i));
		WARN_ON(dev_priv->uncore.fw_domains &
				BIT(FW_DOMAIN_ID_MEDIA_VEBOX0 + i));
		DRM_DEBUG_DRIVER("vecs%u fused off\n", i);
	}
}
