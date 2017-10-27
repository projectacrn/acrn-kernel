/*
 * Copyright © 2014-2017 Intel Corporation
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

#include "intel_guc_ads.h"
#include "intel_uc.h"
#include "i915_drv.h"

/*
 * The Additional Data Struct (ADS) has pointers for different buffers used by
 * the GuC. One single gem object contains the ADS struct itself (guc_ads), the
 * scheduling policies (guc_policies), a structure describing a collection of
 * register sets (guc_mmio_reg_state) and some extra pages for the GuC to save
 * its internal state for sleep. In Gen11, we also have two structures
 * containing HW information and a Command Transport Buffer (to be defined).
 */

static void guc_policy_init(struct guc_policy *policy)
{
	policy->execution_quantum = POLICY_DEFAULT_EXECUTION_QUANTUM_US;
	policy->preemption_time = POLICY_DEFAULT_PREEMPTION_TIME_US;
	policy->fault_time = POLICY_DEFAULT_FAULT_TIME_US;
	policy->policy_flags = 0;
}

static void guc_policies_init(struct guc_policies *policies)
{
	struct guc_policy *policy;
	u32 p, i;

	policies->dpc_promote_time = POLICY_DEFAULT_DPC_PROMOTE_TIME_US;
	policies->max_num_work_items = POLICY_MAX_NUM_WI;

	for (p = 0; p < GUC_CLIENT_PRIORITY_NUM; p++) {
		for (i = GUC_RENDER_ENGINE; i < GUC_MAX_ENGINES_NUM; i++) {
			policy = &policies->policy[p][i];

			guc_policy_init(policy);
		}
	}

	policies->is_valid = 1;
}

static void gen11_guc_policies_init(struct gen11_guc_policies *policies)
{
	struct guc_policy *policy;
	u32 p, class;

	policies->dpc_promote_time = POLICY_DEFAULT_DPC_PROMOTE_TIME_US;
	policies->max_num_work_items = POLICY_MAX_NUM_WI;

	for (p = 0; p < GUC_CLIENT_PRIORITY_NUM; p++) {
		for (class = 0; class < GUC_MAX_ENGINE_CLASSES; class++) {
			policy = &policies->policy[p][class];

			guc_policy_init(policy);
		}
	}

	policies->is_valid = 1;
}

/*
 * The first 80 dwords of the register state context, containing the
 * execlists and ppgtt registers.
 */
#define LR_HW_CONTEXT_SIZE	(80 * sizeof(u32))

int gen11_guc_ads_create(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct i915_vma *vma;
	/* The ads obj includes the struct itself and buffers passed to GuC */
	struct {
		struct gen11_guc_ads ads;
		struct gen11_guc_policies policies;
		struct gen11_guc_mmio_reg_state reg_state;
		struct guc_gt_system_info system_info;
		struct guc_gt_system_additional_info add_system_info;
		struct guc_master_cmd_transport_buffer_alloc ctb_alloc;
		u8 reg_state_buffer[GUC_S3_SAVE_SPACE_PAGES * PAGE_SIZE];
	} __packed *blob;
	const u32 skipped_offset = LRC_HEADER_PAGES * PAGE_SIZE;
	const u32 skipped_size = LRC_PPHWSP_SZ * PAGE_SIZE + LR_HW_CONTEXT_SIZE;
	u32 media_fuse;
	u32 base;
	u32 class;
	int ret;

	GEM_BUG_ON(guc->ads_vma);

	vma = intel_guc_allocate_vma(guc, PAGE_ALIGN(sizeof(*blob)));
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	guc->ads_vma = vma;
	base = intel_guc_ggtt_offset(guc, vma);

	blob = i915_gem_object_pin_map(guc->ads_vma->obj, I915_MAP_WB);
	if (IS_ERR(blob)) {
		ret = PTR_ERR(blob);
		goto err_vma;
	}

	/* GuC scheduling policies */
	gen11_guc_policies_init(&blob->policies);

	/* MMIO reg state */
	for (class = 0; class < GUC_MAX_ENGINE_CLASSES; class++) {
		/* Need the relative offset here, the FW will add the base */
		blob->reg_state.white_list[class].mmio_start =
			RING_FORCE_TO_NONPRIV_REL(0);

		/* Nothing to be saved or restored for now. */
		blob->reg_state.white_list[class].count = 0;
	}

	/*
	 * The GuC requires a "Golden Context" when it reinitialises
	 * engines after a reset. Here we use the Render ring default
	 * context, which must already exist and be pinned in the GGTT,
	 * so its address won't change after we've told the GuC where
	 * to find it. Note that we have to skip our header (1 page),
	 * because our GuC shared data is there.
	 */
	blob->ads.golden_context_lrca[GUC_RENDER_CLASS] =
		intel_guc_ggtt_offset(guc, dev_priv->kernel_context->__engine[RCS].state) +
		skipped_offset;

	/*
	 * We only care about the golden context for the render class, really
	 * (but skipping the execlist part of the context)
	 */
	blob->ads.eng_state_size[GUC_RENDER_CLASS] =
		intel_class_context_size(dev_priv, GUC_RENDER_CLASS) - skipped_size;

	blob->system_info.slice_enabled = hweight8(INTEL_INFO(dev_priv)->sseu.slice_mask);
	blob->system_info.rcs_enabled = 1;
	blob->system_info.bcs_enabled = 1;

	media_fuse = I915_READ(GEN11_GT_VEBOX_VDBOX_DISABLE);
	blob->system_info.vdbox_enable_mask = ~(media_fuse & GEN11_GT_VDBOX_DISABLE_MASK);
	blob->system_info.vebox_enable_mask = ~((media_fuse & GEN11_GT_VEBOX_DISABLE_MASK) >>
						GEN11_GT_VEBOX_DISABLE_SHIFT);

	/*
	 * FIXME: For the moment we just give some space to the GuC so that it
	 * doesn't complain. We will introduce proper Command Transport Buffers
	 * in a future patch.
	*/
	blob->add_system_info.gfx_address_command_transport_pool =
		base + ptr_offset(blob, ctb_alloc);
	blob->add_system_info.command_transport_pool_count = 1;

	blob->ads.scheduler_policies = base + ptr_offset(blob, policies);
	blob->ads.reg_state_buffer = base + ptr_offset(blob, reg_state_buffer);
	blob->ads.reg_state_addr = base + ptr_offset(blob, reg_state);
	blob->ads.gt_system_info = base + ptr_offset(blob, system_info);
	blob->ads.gt_system_additional_info = base + ptr_offset(blob, add_system_info);

	i915_gem_object_unpin_map(guc->ads_vma->obj);

	return 0;

err_vma:
	i915_vma_unpin_and_release(&guc->ads_vma);
	return ret;
}

/**
 * intel_guc_ads_create() - creates GuC ADS
 * @guc: intel_guc struct
 *
 */
int intel_guc_ads_create(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct i915_vma *vma, *kernel_ctx_vma;
	struct page *page;
	/* The ads obj includes the struct itself and buffers passed to GuC */
	struct {
		struct guc_ads ads;
		struct guc_policies policies;
		struct guc_mmio_reg_state reg_state;
		u8 reg_state_buffer[GUC_S3_SAVE_SPACE_PAGES * PAGE_SIZE];
	} __packed *blob;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	const u32 skipped_offset = LRC_HEADER_PAGES * PAGE_SIZE;
	const u32 skipped_size = LRC_PPHWSP_SZ * PAGE_SIZE + LR_HW_CONTEXT_SIZE;
	u32 base;

	GEM_BUG_ON(guc->ads_vma);

	vma = intel_guc_allocate_vma(guc, PAGE_ALIGN(sizeof(*blob)));
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	guc->ads_vma = vma;

	page = i915_vma_first_page(vma);
	blob = kmap(page);

	/* GuC scheduling policies */
	guc_policies_init(&blob->policies);

	/* MMIO reg state */
	for_each_engine(engine, dev_priv, id) {
		blob->reg_state.white_list[engine->guc_id].mmio_start =
			engine->mmio_base + GUC_MMIO_WHITE_LIST_START;

		/* Nothing to be saved or restored for now. */
		blob->reg_state.white_list[engine->guc_id].count = 0;
	}

	/*
	 * The GuC requires a "Golden Context" when it reinitialises
	 * engines after a reset. Here we use the Render ring default
	 * context, which must already exist and be pinned in the GGTT,
	 * so its address won't change after we've told the GuC where
	 * to find it. Note that we have to skip our header (1 page),
	 * because our GuC shared data is there.
	 */
	kernel_ctx_vma = to_intel_context(dev_priv->kernel_context,
					  dev_priv->engine[RCS])->state;
	blob->ads.golden_context_lrca =
		intel_guc_ggtt_offset(guc, kernel_ctx_vma) + skipped_offset;

	/*
	 * The GuC expects us to exclude the portion of the context image that
	 * it skips from the size it is to read. It starts reading from after
	 * the execlist context (so skipping the first page [PPHWSP] and 80
	 * dwords). Weird guc is weird.
	 */
	for_each_engine(engine, dev_priv, id)
		blob->ads.eng_state_size[engine->guc_id] =
			engine->context_size - skipped_size;

	base = intel_guc_ggtt_offset(guc, vma);
	blob->ads.scheduler_policies = base + ptr_offset(blob, policies);
	blob->ads.reg_state_buffer = base + ptr_offset(blob, reg_state_buffer);
	blob->ads.reg_state_addr = base + ptr_offset(blob, reg_state);

	kunmap(page);

	return 0;
}

void intel_guc_ads_destroy(struct intel_guc *guc)
{
	i915_vma_unpin_and_release(&guc->ads_vma);
}
