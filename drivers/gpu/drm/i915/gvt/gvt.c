/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Kevin Tian <kevin.tian@intel.com>
 *    Eddie Dong <eddie.dong@intel.com>
 *
 * Contributors:
 *    Niu Bing <bing.niu@intel.com>
 *    Zhi Wang <zhi.a.wang@intel.com>
 *
 */

#include <linux/types.h>
#include <xen/xen.h>
#include <linux/kthread.h>

#include "i915_drv.h"
#include "gvt.h"

struct intel_gvt_host intel_gvt_host;

static const char * const supported_hypervisors[] = {
	[INTEL_GVT_HYPERVISOR_XEN] = "XEN",
	[INTEL_GVT_HYPERVISOR_KVM] = "KVM",
	[INTEL_GVT_HYPERVISOR_ACRN] = "ACRN",
};

static const struct intel_gvt_ops intel_gvt_ops = {
	.emulate_cfg_read = intel_vgpu_emulate_cfg_read,
	.emulate_cfg_write = intel_vgpu_emulate_cfg_write,
	.emulate_mmio_read = intel_vgpu_emulate_mmio_read,
	.emulate_mmio_write = intel_vgpu_emulate_mmio_write,
	.vgpu_create = intel_gvt_create_vgpu,
	.vgpu_destroy = intel_gvt_destroy_vgpu,
	.vgpu_reset = intel_gvt_reset_vgpu,
	.vgpu_activate = intel_gvt_activate_vgpu,
	.vgpu_deactivate = intel_gvt_deactivate_vgpu,
};

/**
 * intel_gvt_init_host - Load MPT modules and detect if we're running in host
 * @gvt: intel gvt device
 *
 * This function is called at the driver loading stage. If failed to find a
 * loadable MPT module or detect currently we're running in a VM, then GVT-g
 * will be disabled
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_gvt_init_host(void)
{
	if (intel_gvt_host.initialized)
		return 0;

	/* Xen DOM U */
	if (xen_domain() && !xen_initial_domain())
		return -ENODEV;

	/* Try to load MPT modules for hypervisors */
	if (xen_initial_domain()) {
		/* In Xen dom0 */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(xengt_mpt), "xengt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_XEN;
	} else {
#if IS_ENABLED(CONFIG_DRM_I915_GVT_KVMGT)
		/* not in Xen. Try KVMGT */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(kvmgt_mpt), "kvmgt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_KVM;
#endif
		/* not in Xen. Try ACRN */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(acrn_gvt_mpt), "acrn-gvt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_ACRN;
		printk("acrn-gvt %s\n", intel_gvt_host.mpt?"found":"not found");
	}

	/* Fail to load MPT modules - bail out */
	if (!intel_gvt_host.mpt)
		return -EINVAL;

	gvt_dbg_core("Running with hypervisor %s in host mode\n",
			supported_hypervisors[intel_gvt_host.hypervisor_type]);

	intel_gvt_host.initialized = true;
	return 0;
}

static void init_device_info(struct intel_gvt *gvt)
{
	struct intel_gvt_device_info *info = &gvt->device_info;
	struct pci_dev *pdev = gvt->dev_priv->drm.pdev;

	if (IS_BROADWELL(gvt->dev_priv) || IS_SKYLAKE(gvt->dev_priv)
		|| IS_BROXTON(gvt->dev_priv)
		|| IS_KABYLAKE(gvt->dev_priv)) {
		info->max_support_vgpus = 8;
		info->cfg_space_size = 256;
		info->mmio_size = 2 * 1024 * 1024;
		/* order of mmio size. assert(2^order == mmio_size) */
		info->mmio_size_order = 9;
		info->mmio_bar = 0;
		info->gtt_start_offset = 8 * 1024 * 1024;
		info->gtt_entry_size = 8;
		info->gtt_entry_size_shift = 3;
		info->gmadr_bytes_in_cmd = 8;
		info->max_surface_size = 36 * 1024 * 1024;
	}
	info->msi_cap_offset = pdev->msi_cap;
}

static int gvt_service_thread(void *data)
{
	struct intel_gvt *gvt = (struct intel_gvt *)data;
	int ret;

	gvt_dbg_core("service thread start\n");

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(gvt->service_thread_wq,
				kthread_should_stop() || gvt->service_request);

		if (kthread_should_stop())
			break;

		if (WARN_ONCE(ret, "service thread is waken up by signal.\n"))
			continue;

		if (test_and_clear_bit(INTEL_GVT_REQUEST_EMULATE_VBLANK,
					(void *)&gvt->service_request)) {
			mutex_lock(&gvt->lock);
			intel_gvt_emulate_vblank(gvt);
			mutex_unlock(&gvt->lock);
		}

		if (test_bit(INTEL_GVT_REQUEST_SCHED,
				(void *)&gvt->service_request) ||
			test_bit(INTEL_GVT_REQUEST_EVENT_SCHED,
					(void *)&gvt->service_request)) {
			intel_gvt_schedule(gvt);
		}
	}

	return 0;
}

static void clean_service_thread(struct intel_gvt *gvt)
{
	kthread_stop(gvt->service_thread);
}

static int init_service_thread(struct intel_gvt *gvt)
{
	init_waitqueue_head(&gvt->service_thread_wq);

	gvt->service_thread = kthread_run(gvt_service_thread,
			gvt, "gvt_service_thread");
	if (IS_ERR(gvt->service_thread)) {
		gvt_err("fail to start service thread.\n");
		return PTR_ERR(gvt->service_thread);
	}
	return 0;
}

void intel_gvt_init_pipe_info(struct intel_gvt *gvt);

/*
 * When enabling multi-plane in DomU, an issue is that the PLANE_BUF_CFG
 * register cannot be updated dynamically, since Dom0 has no idea of the
 * plane information of DomU's planes, so here we allocate the
 * ddb entries for all the possible enabled planes.
 */
void intel_gvt_allocate_ddb(struct intel_gvt *gvt,
		struct skl_ddb_allocation *ddb, unsigned int active_crtcs)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	unsigned int pipe_size, ddb_size, plane_size, plane_cnt;
	u16 start, end;
	enum pipe pipe;
	enum plane_id plane;
	int i = 0;
	int num_active = hweight32(active_crtcs);

	if (WARN_ON(!num_active))
		return;

	ddb_size = INTEL_INFO(dev_priv)->ddb_size;
	ddb_size -= 4; /* 4 blocks for bypass path allocation */
	pipe_size = ddb_size / num_active;

	memset(ddb, 0, sizeof(*ddb));
	for_each_pipe_masked(dev_priv, pipe, active_crtcs) {
		start = pipe_size * (i++);
		end = start + pipe_size;
		ddb->plane[pipe][PLANE_CURSOR].start = end - 8;
		ddb->plane[pipe][PLANE_CURSOR].end = end;

		plane_cnt = (INTEL_INFO(dev_priv)->num_sprites[pipe] + 1);
		plane_size = (pipe_size - 8) / plane_cnt;

		for_each_universal_plane(dev_priv, pipe, plane) {
			ddb->plane[pipe][plane].start = start +
				(plane * (pipe_size - 8) / plane_cnt);
			ddb->plane[pipe][plane].end =
				ddb->plane[pipe][plane].start + plane_size;
		}
	}
}

/**
 * intel_gvt_clean_device - clean a GVT device
 * @gvt: intel gvt device
 *
 * This function is called at the driver unloading stage, to free the
 * resources owned by a GVT device.
 *
 */
void intel_gvt_clean_device(struct drm_i915_private *dev_priv)
{
	struct intel_gvt *gvt = to_gvt(dev_priv);

	if (WARN_ON(!gvt))
		return;

	clean_service_thread(gvt);
	intel_gvt_clean_cmd_parser(gvt);
	intel_gvt_clean_sched_policy(gvt);
	intel_gvt_clean_workload_scheduler(gvt);
	intel_gvt_clean_opregion(gvt);
	intel_gvt_clean_gtt(gvt);
	intel_gvt_clean_irq(gvt);
	intel_gvt_clean_mmio_info(gvt);
	intel_gvt_free_firmware(gvt);

	intel_gvt_hypervisor_host_exit(&dev_priv->drm.pdev->dev, gvt);
	intel_gvt_clean_vgpu_types(gvt);

	idr_destroy(&gvt->vgpu_idr);

	intel_gvt_destroy_idle_vgpu(gvt->idle_vgpu);

	kfree(dev_priv->gvt);
	dev_priv->gvt = NULL;
}

#define BITS_PER_DOMAIN 4
#define MAX_PLANES_PER_DOMAIN 4
#define DOMAIN_PLANE_OWNER(owner, pipe, plane) \
		((((owner) >> (pipe) * BITS_PER_DOMAIN * MAX_PLANES_PER_DOMAIN) >>  \
		  BITS_PER_DOMAIN * (plane)) & 0xf)

/**
 * intel_gvt_init_device - initialize a GVT device
 * @dev_priv: drm i915 private data
 *
 * This function is called at the initialization stage, to initialize
 * necessary GVT components.
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_gvt_init_device(struct drm_i915_private *dev_priv)
{
	struct intel_gvt *gvt;
	struct intel_vgpu *vgpu;
	int ret;

	/*
	 * Cannot initialize GVT device without intel_gvt_host gets
	 * initialized first.
	 */
	if (WARN_ON(!intel_gvt_host.initialized))
		return -EINVAL;

	if (WARN_ON(dev_priv->gvt))
		return -EEXIST;

	gvt = kzalloc(sizeof(struct intel_gvt), GFP_KERNEL);
	if (!gvt)
		return -ENOMEM;

	gvt_dbg_core("init gvt device\n");

	idr_init(&gvt->vgpu_idr);
	spin_lock_init(&gvt->scheduler.mmio_context_lock);
	mutex_init(&gvt->lock);
	mutex_init(&gvt->sched_lock);
	gvt->dev_priv = dev_priv;

	init_device_info(gvt);

	ret = intel_gvt_setup_mmio_info(gvt);
	if (ret)
		goto out_clean_idr;

	ret = intel_gvt_load_firmware(gvt);
	if (ret)
		goto out_clean_mmio_info;

	ret = intel_gvt_init_irq(gvt);
	if (ret)
		goto out_free_firmware;

	ret = intel_gvt_init_gtt(gvt);
	if (ret)
		goto out_clean_irq;

	ret = intel_gvt_init_opregion(gvt);
	if (ret)
		goto out_clean_gtt;

	ret = intel_gvt_init_workload_scheduler(gvt);
	if (ret)
		goto out_clean_opregion;

	ret = intel_gvt_init_sched_policy(gvt);
	if (ret)
		goto out_clean_workload_scheduler;

	ret = intel_gvt_init_cmd_parser(gvt);
	if (ret)
		goto out_clean_sched_policy;

	ret = init_service_thread(gvt);
	if (ret)
		goto out_clean_cmd_parser;

	ret = intel_gvt_init_vgpu_types(gvt);
	if (ret)
		goto out_clean_thread;

	intel_gvt_init_pipe_info(gvt);

	ret = intel_gvt_hypervisor_host_init(&dev_priv->drm.pdev->dev, gvt,
				&intel_gvt_ops);
	if (ret) {
		gvt_err("failed to register gvt-g host device: %d\n", ret);
		goto out_clean_types;
	}

	vgpu = intel_gvt_create_idle_vgpu(gvt);
	if (IS_ERR(vgpu)) {
		ret = PTR_ERR(vgpu);
		gvt_err("failed to create idle vgpu\n");
		goto out_clean_types;
	}
	gvt->idle_vgpu = vgpu;

	dev_priv->gvt = gvt;

	if (i915_modparams.avail_planes_per_pipe) {
		unsigned long long domain_plane_owners;
		int plane;
		enum pipe pipe;

		/*
		 * Each nibble represents domain id
		 * ids can be from 0-F. 0 for Dom0, 1,2,3...0xF for DomUs
		 * plane_owner[i] holds the id of the domain that owns it,eg:0,1,2 etc
		 */
		domain_plane_owners = i915_modparams.domain_plane_owners;
		for_each_pipe(dev_priv, pipe) {
			for_each_universal_plane(dev_priv, pipe, plane) {
				gvt->pipe_info[pipe].plane_owner[plane] =
					DOMAIN_PLANE_OWNER(domain_plane_owners, pipe, plane);
			}
		}
	}

	gvt_dbg_core("gvt device initialization is done\n");
	return 0;

out_clean_types:
	intel_gvt_clean_vgpu_types(gvt);
out_clean_thread:
	clean_service_thread(gvt);
out_clean_cmd_parser:
	intel_gvt_clean_cmd_parser(gvt);
out_clean_sched_policy:
	intel_gvt_clean_sched_policy(gvt);
out_clean_workload_scheduler:
	intel_gvt_clean_workload_scheduler(gvt);
out_clean_opregion:
	intel_gvt_clean_opregion(gvt);
out_clean_gtt:
	intel_gvt_clean_gtt(gvt);
out_clean_irq:
	intel_gvt_clean_irq(gvt);
out_free_firmware:
	intel_gvt_free_firmware(gvt);
out_clean_mmio_info:
	intel_gvt_clean_mmio_info(gvt);
out_clean_idr:
	idr_destroy(&gvt->vgpu_idr);
	kfree(gvt);
	return ret;
}

int gvt_pause_user_domains(struct drm_i915_private *dev_priv)
{
	struct intel_vgpu *vgpu;
	int id, ret = 0;

	if (!intel_gvt_active(dev_priv))
		return 0;

	for_each_active_vgpu(dev_priv->gvt, vgpu, id) {
		ret = intel_gvt_hypervisor_pause_domain(vgpu);
	}

	return ret;
}

int gvt_unpause_user_domains(struct drm_i915_private *dev_priv)
{
	struct intel_vgpu *vgpu;
	int id, ret = 0;

	if (!intel_gvt_active(dev_priv))
		return 0;

	for_each_active_vgpu(dev_priv->gvt, vgpu, id) {
		ret = intel_gvt_hypervisor_unpause_domain(vgpu);
	}

	return ret;
}

int gvt_dom0_ready(struct drm_i915_private *dev_priv)
{
	if (!intel_gvt_active(dev_priv))
		return 0;

	return intel_gvt_hypervisor_dom0_ready();
}
