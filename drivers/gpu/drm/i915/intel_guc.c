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

#include "intel_guc.h"
#include "intel_guc_submission.h"
#include "i915_drv.h"

static void gen8_guc_raise_irq(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	I915_WRITE(GUC_SEND_INTERRUPT, GUC_SEND_TRIGGER);
}

static void gen11_guc_raise_irq(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	I915_WRITE(GEN11_GUC_HOST_INTERRUPT, GUC_SEND_TRIGGER);
}

static inline i915_reg_t guc_send_reg(struct intel_guc *guc, u32 i)
{
	GEM_BUG_ON(!guc->send_regs.base);
	GEM_BUG_ON(!guc->send_regs.count);
	GEM_BUG_ON(i >= guc->send_regs.count);

	return _MMIO(guc->send_regs.base + 4 * i);
}

void intel_guc_init_send_regs(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	enum forcewake_domains fw_domains = 0;
	unsigned int i;

	/* For GuC CT we need to use Gen11 registers */
	if (HAS_GUC_CT(dev_priv)) {
		guc->send_regs.base =
				i915_mmio_reg_offset(GEN11_SOFT_SCRATCH(0));
		guc->send_regs.count = GEN11_SOFT_SCRATCH_COUNT;
	} else {
		guc->send_regs.base = i915_mmio_reg_offset(SOFT_SCRATCH(0));
		guc->send_regs.count = SOFT_SCRATCH_COUNT - 1;
	}

	for (i = 0; i < guc->send_regs.count; i++) {
		fw_domains |= intel_uncore_forcewake_for_reg(dev_priv,
					guc_send_reg(guc, i),
					FW_REG_READ | FW_REG_WRITE);
	}
	guc->send_regs.fw_domains = fw_domains;
}

void intel_guc_init_early(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	intel_guc_ct_init_early(&guc->ct);

	mutex_init(&guc->send_mutex);
	guc->send = intel_guc_send_nop;
	guc->recv = intel_guc_receive_nop;
	if (INTEL_GEN(dev_priv) >= 11)
		guc->notify = gen11_guc_raise_irq;
	else
		guc->notify = gen8_guc_raise_irq;
}

static u32 get_gt_type(struct drm_i915_private *dev_priv)
{
	/* XXX: GT type based on PCI device ID? field seems unused by fw */
	return 0;
}

static u32 get_core_family(struct drm_i915_private *dev_priv)
{
	u32 gen = INTEL_GEN(dev_priv);

	switch (gen) {
	case 9:
		return GUC_CORE_FAMILY_GEN9;
	case 11:
		return GUC_CORE_FAMILY_GEN11;
	default:
		MISSING_CASE(gen);
		return GUC_CORE_FAMILY_UNKNOWN;
	}
}

static u32 guc_feature_flags(struct drm_i915_private *dev_priv)
{
	u32 feature_flags = 0;

	if (INTEL_GEN(dev_priv) <= 10) {
		feature_flags |= GUC_CTL_VCS2_ENABLED;
		if (i915_modparams.enable_guc_submission)
			feature_flags |= GUC_CTL_KERNEL_SUBMISSIONS;
		else
			feature_flags |= GUC_CTL_DISABLE_SCHEDULER;
	}

	return feature_flags;
}

/*
 * Initialise the GuC parameter block before starting the firmware
 * transfer. These parameters are read by the firmware on startup
 * and cannot be changed thereafter.
 */
void intel_guc_init_params(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	u32 params[GUC_CTL_MAX_DWORDS];
	int i;

	memset(params, 0, sizeof(params));

	params[GUC_CTL_DEVICE_INFO] |=
		(get_gt_type(dev_priv) << GUC_CTL_GT_TYPE_SHIFT) |
		(get_core_family(dev_priv) << GUC_CTL_CORE_FAMILY_SHIFT);

	/*
	 * GuC ARAT increment is 10 ns. GuC default scheduler quantum is one
	 * second. This ARAR is calculated by:
	 * Scheduler-Quantum-in-ns / ARAT-increment-in-ns = 1000000000 / 10
	 */
	params[GUC_CTL_ARAT_HIGH] = 0;
	params[GUC_CTL_ARAT_LOW] = 100000000;

	params[GUC_CTL_WA] |= GUC_CTL_WA_UK_BY_DRIVER;

	params[GUC_CTL_FEATURE] |= guc_feature_flags(dev_priv);

	params[GUC_CTL_LOG_PARAMS] = guc->log.flags;

	if (i915_modparams.guc_log_level >= 0) {
		params[GUC_CTL_DEBUG] =
			i915_modparams.guc_log_level << GUC_LOG_VERBOSITY_SHIFT;
	} else {
		params[GUC_CTL_DEBUG] = GUC_LOG_DISABLED;
	}

	if (i915_modparams.enable_guc_submission) {
		u32 ads = guc_ggtt_offset(guc->ads_vma) >> PAGE_SHIFT;
		u32 pgs = guc_ggtt_offset(dev_priv->guc.stage_desc_pool) >> PAGE_SHIFT;
		u32 ctx_in_16 = guc->max_stage_desc / 16;

		if (INTEL_GEN(dev_priv) >= 11) {
			params[GEN11_GUC_ADS_CTL] = GEN11_GUC_ADS_ENABLE;
			params[GEN11_GUC_ADS_CTL] |= ads << GEN11_GUC_ADS_ADDR_SHIFT;
		} else {
			params[GUC_CTL_DEBUG] |= GUC_ADS_ENABLED;
			params[GUC_CTL_DEBUG] |= ads << GUC_ADS_ADDR_SHIFT;
		}

		params[GUC_CTL_CTXINFO] = (pgs << GUC_CTL_BASE_ADDR_SHIFT) |
			(ctx_in_16 << GUC_CTL_CTXNUM_IN16_SHIFT);
	}

	/*
	 * All SOFT_SCRATCH registers are in FORCEWAKE_BLITTER domain and
	 * they are power context saved so it's ok to release forcewake
	 * when we are done here and take it again at xfer time.
	 */
	intel_uncore_forcewake_get(dev_priv, FORCEWAKE_BLITTER);

	I915_WRITE(SOFT_SCRATCH(0), 0);

	for (i = 0; i < GUC_CTL_MAX_DWORDS; i++)
		I915_WRITE(SOFT_SCRATCH(1 + i), params[i]);

	intel_uncore_forcewake_put(dev_priv, FORCEWAKE_BLITTER);
}

int intel_guc_send_nop(struct intel_guc *guc, const u32 *action, u32 len,
		       u32 *response)
{
	WARN(1, "Unexpected send: action=%#x\n", *action);
	return -ENODEV;
}

void intel_guc_receive_nop(struct intel_guc *guc)
{
	WARN(1, "Unexpected receive\n");
}

/*
 * This function implements the MMIO based host to GuC interface.
 */
int intel_guc_send_mmio(struct intel_guc *guc, const u32 *action, u32 len,
			u32 *response)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	u32 status;
	int i;
	int ret;

	GEM_BUG_ON(!len);
	GEM_BUG_ON(len > guc->send_regs.count);

	/* If CT is available, we expect to use MMIO only during init/fini */
	GEM_BUG_ON(HAS_GUC_CT(dev_priv) &&
		*action != INTEL_GUC_ACTION_REGISTER_COMMAND_TRANSPORT_BUFFER &&
		*action != INTEL_GUC_ACTION_DEREGISTER_COMMAND_TRANSPORT_BUFFER);

	mutex_lock(&guc->send_mutex);
	intel_uncore_forcewake_get(dev_priv, guc->send_regs.fw_domains);

	for (i = 0; i < len; i++)
		I915_WRITE(guc_send_reg(guc, i), action[i]);

	POSTING_READ(guc_send_reg(guc, i - 1));

	intel_guc_notify(guc);

	/*
	 * No GuC command should ever take longer than 10ms.
	 * Fast commands should still complete in 10us.
	 */
	ret = __intel_wait_for_register_fw(dev_priv,
					   guc_send_reg(guc, 0),
					   INTEL_GUC_RECV_MASK,
					   INTEL_GUC_RECV_MASK,
					   10, 10, &status);
	if (INTEL_GUC_RECV_TO_STATUS(status) != INTEL_GUC_STATUS_SUCCESS) {
		/*
		 * Either the GuC explicitly returned an error (which
		 * we convert to -EIO here) or no response at all was
		 * received within the timeout limit (-ETIMEDOUT)
		 */
		if (ret != -ETIMEDOUT)
			ret = -EIO;

		DRM_WARN("INTEL_GUC_SEND: Action 0x%X failed;"
			 " ret=%d status=0x%08X response=0x%08X\n",
			 action[0], ret, status, I915_READ(SOFT_SCRATCH(15)));
	} else {
		if (response) {
			/* Skip reg[0] with the status/response mask */
			for (i = 1; i < guc->send_regs.count; i++)
				response[i] = I915_READ(guc_send_reg(guc, i));
		}

		/* Use data encoded by Guc in status dword as return value */
		ret = INTEL_GUC_RECV_TO_DATA(status);
	}

	intel_uncore_forcewake_put(dev_priv, guc->send_regs.fw_domains);
	mutex_unlock(&guc->send_mutex);

	return ret;
}

/*
 * This function implements the MMIO based GuC to host interface.
 */
void intel_guc_receive_mmio(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	u32 msg, flush;

	/*
	 * Sample the log buffer flush related bits & clear them out now
	 * itself from the message identity register to minimize the
	 * probability of losing a flush interrupt, when there are back
	 * to back flush interrupts.
	 * There can be a new flush interrupt, for different log buffer
	 * type (like for ISR), whilst Host is handling one (for DPC).
	 * Since same bit is used in message register for ISR & DPC, it
	 * could happen that GuC sets the bit for 2nd interrupt but Host
	 * clears out the bit on handling the 1st interrupt.
	 */

	msg = I915_READ(SOFT_SCRATCH(15));
	flush = msg & (INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED |
		       INTEL_GUC_RECV_MSG_FLUSH_LOG_BUFFER);
	if (flush) {
		/* Clear the message bits that are handled */
		I915_WRITE(SOFT_SCRATCH(15), msg & ~flush);

		intel_guc_log_flush(&dev_priv->guc);
	} else {
		/*
		 * Not clearing of unhandled event bits won't result in
		 * re-triggering of the interrupt.
		 */
	}
}

void intel_guc_notification_handler(struct intel_guc *guc)
{
	guc->recv(guc);
}

int intel_guc_sample_forcewake(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	u32 action[2];

	action[0] = INTEL_GUC_ACTION_SAMPLE_FORCEWAKE;
	/* WaRsDisableCoarsePowerGating:skl,bxt */
	if (!intel_rc6_enabled() ||
	    NEEDS_WaRsDisableCoarsePowerGating(dev_priv))
		action[1] = 0;
	else
		/* bit 0 and 1 are for Render and Media domain separately */
		action[1] = GUC_FORCEWAKE_RENDER | GUC_FORCEWAKE_MEDIA;

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

/**
 * intel_guc_auth_huc() - Send action to GuC to authenticate HuC ucode
 * @guc: intel_guc structure
 * @rsa_offset: rsa offset w.r.t ggtt base of huc vma
 *
 * Triggers a HuC firmware authentication request to the GuC via intel_guc_send
 * INTEL_GUC_ACTION_AUTHENTICATE_HUC interface. This function is invoked by
 * intel_huc_auth().
 *
 * Return:	non-zero code on error
 */
int intel_guc_auth_huc(struct intel_guc *guc, u32 rsa_offset)
{
	u32 action[] = {
		INTEL_GUC_ACTION_AUTHENTICATE_HUC,
		rsa_offset
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

/**
 * intel_guc_suspend() - notify GuC entering suspend state
 * @dev_priv:	i915 device private
 */
int intel_guc_suspend(struct drm_i915_private *dev_priv)
{
	struct intel_guc *guc = &dev_priv->guc;
	u32 data[3];

	if (guc->fw.load_status != INTEL_UC_FIRMWARE_SUCCESS)
		return 0;

	guc->interrupts.disable(dev_priv);

	data[0] = INTEL_GUC_ACTION_ENTER_S_STATE;
	/* any value greater than GUC_POWER_D0 */
	data[1] = GUC_POWER_D1;
	data[2] = guc_ggtt_offset(guc->shared_data);

	return intel_guc_send(guc, data, ARRAY_SIZE(data));
}

/**
 * intel_guc_reset_engine() - ask GuC to reset an engine
 * @guc:	intel_guc structure
 * @engine:	engine to be reset
 */
int intel_guc_reset_engine(struct intel_guc *guc,
			   struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct intel_guc_client *client = guc->execbuf_client;
	u32 data[7];
	u32 len;

	GEM_BUG_ON(!client);

	data[0] = INTEL_GUC_ACTION_REQUEST_ENGINE_RESET;
	if (INTEL_GEN(dev_priv) < 11) {
		data[1] = engine->guc_id;
		data[2] = 0; /* options */
		data[3] = 0; /* pagefault recovery in progress?*/
		data[4] = 0; /* unused */
		data[5] = client->stage_id;
		data[6] = guc_ggtt_offset(guc->shared_data);
		len = 7;
	} else {
		data[1] = GEN11_GUC_ENGINE_ID_FOR_H2G(engine->guc_id);
		data[2] = 0; /* options */
		data[3] = client->stage_id;
		data[4] = guc_ggtt_offset(guc->shared_data);
		len = 5;
	}

	return intel_guc_send(guc, data, len);
}

/**
 * intel_guc_resume() - notify GuC resuming from suspend state
 * @dev_priv:	i915 device private
 */
int intel_guc_resume(struct drm_i915_private *dev_priv)
{
	struct intel_guc *guc = &dev_priv->guc;
	u32 data[3];

	if (guc->fw.load_status != INTEL_UC_FIRMWARE_SUCCESS)
		return 0;

	if (HAS_GUC_CT(dev_priv) || i915_modparams.guc_log_level >= 0)
		guc->interrupts.enable(dev_priv);

	data[0] = INTEL_GUC_ACTION_EXIT_S_STATE;
	data[1] = GUC_POWER_D0;
	data[2] = guc_ggtt_offset(guc->shared_data);

	return intel_guc_send(guc, data, ARRAY_SIZE(data));
}

/**
 * intel_guc_allocate_vma() - Allocate a GGTT VMA for GuC usage
 * @guc:	the guc
 * @size:	size of area to allocate (both virtual space and memory)
 *
 * This is a wrapper to create an object for use with the GuC. In order to
 * use it inside the GuC, an object needs to be pinned lifetime, so we allocate
 * both some backing storage and a range inside the Global GTT. We must pin
 * it in the GGTT somewhere other than than [0, GUC_WOPCM_TOP) because that
 * range is reserved inside GuC.
 *
 * Return:	A i915_vma if successful, otherwise an ERR_PTR.
 */
struct i915_vma *intel_guc_allocate_vma(struct intel_guc *guc, u32 size)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int ret;

	obj = i915_gem_object_create(dev_priv, size);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	vma = i915_vma_instance(obj, &dev_priv->ggtt.base, NULL);
	if (IS_ERR(vma))
		goto err;

	ret = i915_vma_pin(vma, 0, PAGE_SIZE,
			   PIN_GLOBAL | PIN_OFFSET_BIAS | GUC_WOPCM_TOP);
	if (ret) {
		vma = ERR_PTR(ret);
		goto err;
	}

	return vma;

err:
	i915_gem_object_put(obj);
	return vma;
}

u32 intel_guc_wopcm_size(struct drm_i915_private *dev_priv)
{
	u32 wopcm_size = GUC_WOPCM_TOP;

	/* On BXT, the top of WOPCM is reserved for RC6 context */
	if (IS_GEN9_LP(dev_priv))
		wopcm_size -= BXT_GUC_WOPCM_RC6_RESERVED;

	return wopcm_size;
}

void intel_guc_process_default_action(struct intel_guc *guc, u32 msg)
{
	if (msg & (INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED |
		   INTEL_GUC_RECV_MSG_FLUSH_LOG_BUFFER))
		intel_guc_log_flush(guc);
}
