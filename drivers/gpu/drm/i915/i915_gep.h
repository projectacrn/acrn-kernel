/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _I915_GEP_H_
#define _I915_GEP_H_

#define I915_GEP_LOG_MAX	256

struct drm_i915_gem_request;
struct drm_device;

struct i915_gep_buf_entry {
	u64 start_time;
	u64 end_time;
};

struct i915_gep_req {
	pid_t pid;
	int vgpu_id;
	u64 perf_tag;
	struct i915_gep_buf_entry *cpu_addr;
	u64 gpu_addr;
};

bool i915_gep_is_enabled(void);

int i915_gep_enable(struct drm_device *dev, bool enable);

int i915_gep_start_task(struct drm_i915_gem_request *req);

int i915_gep_end_task(struct drm_i915_gem_request *req);

void i915_gep_init_req(struct drm_i915_gem_request *req, int vgpu_id);

void i915_gep_read_req(struct drm_i915_gem_request *req);

void i915_gep_trace(const char *fmt, ...);

void i915_gep_start_trace(const char *fmt, ...);

void i915_gep_end_trace(void);

#endif
