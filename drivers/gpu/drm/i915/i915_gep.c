// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <drm/drm.h>
#include "i915_drv.h"

struct i915_gep_info {
	struct drm_i915_gem_object *buf_obj;
	int buf_offset;
	u64 ggtt_offset;
	char __iomem *cpu_addr;
	bool enabled;
};
struct i915_gep_info i915_gep;

#define CIRCLE_BUF_SIZE (256 * PAGE_SIZE)
#define PIPE_CONTROL_WRITE_TIMESTAMP  (0x03 << 14)
#define FLUSH_DW_WRITE_TIMESTAMP      (0x03 << 14)
#define END_TIME_OFFSET 8

static int i915_gep_init(struct drm_device *dev)
{
	int ret;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_vma *vma;

	i915_gep.buf_obj = i915_gem_object_create(dev_priv, CIRCLE_BUF_SIZE);
	if (i915_gep.buf_obj == NULL) {
		DRM_ERROR("Failed to allocate gep bo\n");
		return -ENOMEM;
	}

	vma = i915_gem_object_ggtt_pin(i915_gep.buf_obj, NULL, 0,
					PAGE_SIZE, PIN_MAPPABLE);
	if (IS_ERR(vma)) {
		DRM_ERROR("Failed to pin gep bo\n");
		goto err_unref;
	}

	i915_gep.ggtt_offset = i915_ggtt_offset(vma);
	i915_gep.cpu_addr = io_mapping_map_wc(&dev_priv->ggtt.mappable,
					i915_gep.ggtt_offset, CIRCLE_BUF_SIZE);

	if (i915_gep.cpu_addr == NULL) {
		DRM_ERROR("Failed to pin gep bo\n");
		ret = -ENOSPC;
		goto err_unpin;
	}

	i915_gep.enabled = true;
	i915_gep.buf_offset = 0;

	return 0;

err_unpin:
	__i915_vma_unpin(vma);
err_unref:
	drm_gem_object_unreference(&i915_gep.buf_obj->base);
	return ret;
}

static int i915_gep_exit(void)
{
	i915_gem_free_object(&i915_gep.buf_obj->base);
	iounmap(i915_gep.cpu_addr);
	i915_gep.buf_obj = NULL;
	i915_gep.cpu_addr = NULL;
	i915_gep.ggtt_offset = 0;
	i915_gep.enabled = false;
	i915_gep.buf_offset = 0;
	return 0;
}

static void i915_gep_get_buf_space(struct drm_i915_gem_request *req)
{
	if (i915_gep.buf_offset + sizeof(struct i915_gep_buf_entry) >=
						CIRCLE_BUF_SIZE)
		i915_gep.buf_offset = 0;

	req->gep_req.cpu_addr = (struct i915_gep_buf_entry *)
			(i915_gep.cpu_addr + i915_gep.buf_offset);
	req->gep_req.gpu_addr = i915_gep.ggtt_offset + i915_gep.buf_offset;
	i915_gep.buf_offset += sizeof(struct i915_gep_buf_entry);
}

static void i915_gep_mi_pipe_control(struct drm_i915_gem_request *req,
				     u_int32_t addr)
{
	u32 *cs;

	cs = intel_ring_begin(req, 6);
	if (IS_ERR(cs)) {
		DRM_ERROR("Failed to alloc ring\n");
		return;
	}

	*cs++ = GFX_OP_PIPE_CONTROL(6);
	*cs++ = PIPE_CONTROL_WRITE_TIMESTAMP |
			PIPE_CONTROL_GLOBAL_GTT_IVB;
	*cs++ = addr;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	intel_ring_advance(req, cs);
}

static void i915_gep_mi_flush_dw(struct drm_i915_gem_request *req,
				 u_int32_t addr)
{
	u32 *cs;

	cs = intel_ring_begin(req, 6);
	if (IS_ERR(cs)) {
		DRM_ERROR("Failed to alloc ring\n");
		return;
	}

	*cs++ = (MI_FLUSH_DW + 2) | FLUSH_DW_WRITE_TIMESTAMP;
	*cs++ = (addr & 0xFFFFFFF8) | MI_FLUSH_DW_USE_GTT;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	intel_ring_advance(req, cs);

}

void i915_gep_init_req(struct drm_i915_gem_request *req, int vgpu_id)
{
	req->gep_req.pid = current->pid;
	req->gep_req.vgpu_id = vgpu_id;
}

void i915_gep_read_req(struct drm_i915_gem_request *req)
{

	if (i915_gep.enabled && req->gep_req.cpu_addr != NULL) {
		struct intel_engine_cs *engine = req->engine;
		struct drm_i915_private *dev_priv = engine->i915;
		u_int64_t gpu_time = I915_READ64_2x32(
				RING_TIMESTAMP(engine->mmio_base),
				RING_TIMESTAMP_UDW(engine->mmio_base));
		i915_gep_trace("i915_gep_read_req pid=%d vgpu_id=%d hw_ctx=%d fence_ctx=%llu seqno=%u global_seqno=%u engine=%d prio=%d gpu_time=%llx start=%llx end=%llx",
			req->gep_req.pid, req->gep_req.vgpu_id,
			req->ctx->hw_id, req->fence.context,
			req->fence.seqno, req->global_seqno,
			req->engine->id, req->priotree.priority, gpu_time,
			req->gep_req.cpu_addr->start_time,
			req->gep_req.cpu_addr->end_time);
	}
}

int i915_gep_start_task(struct drm_i915_gem_request *req)
{
	if (!i915_gep.enabled)
		return 0;

	i915_gep_get_buf_space(req);

	if (req->engine->id == RCS)
		i915_gep_mi_pipe_control(req, req->gep_req.gpu_addr);
	else
		i915_gep_mi_flush_dw(req, req->gep_req.gpu_addr);
	return 0;
}

int i915_gep_end_task(struct drm_i915_gem_request *req)
{
	if (!i915_gep.enabled)
		return 0;

	if (req->engine->id == RCS)
		i915_gep_mi_pipe_control(req, req->gep_req.gpu_addr +
					END_TIME_OFFSET);
	else
		i915_gep_mi_flush_dw(req, req->gep_req.gpu_addr +
					END_TIME_OFFSET);

	return 0;
}

bool i915_gep_is_enabled(void)
{
	return i915_gep.enabled;
}

int i915_gep_enable(struct drm_device *dev, bool enable)
{
	if (!i915_gep.enabled && enable)
		return i915_gep_init(dev);
	if (i915_gep.enabled && !enable)
		return i915_gep_exit();
	return 0;
}

void i915_gep_trace(const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!i915_gep.enabled)
		return;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	trace_gep_log(&vaf);
	va_end(args);
}

void i915_gep_start_trace(const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int len;
	char str[I915_GEP_LOG_MAX];

	if (!i915_gep.enabled)
		return;

	len = snprintf(str, I915_GEP_LOG_MAX, "B|%d|", current->tgid);

	va_start(args, fmt);
	vsnprintf(str + len, I915_GEP_LOG_MAX - len, fmt, args);
	vaf.fmt = str;
	vaf.va = &args;
	trace_gep_log(&vaf);
	va_end(args);
}

void i915_gep_end_trace(void)
{
	i915_gep_trace("E");
}
