// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uio.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <linux/vbs/vq.h>
#include <linux/hashtable.h>

#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-be-bridge.h"
#include "intel-ipu4-virtio-be.h"
#include "intel-ipu4-virtio-be-request-queue.h"

/**
 * struct ipu4_virtio_be_priv - Backend of virtio-rng based on VBS-K
 *
 * @dev		: instance of struct virtio_dev_info
 * @vqs		: instances of struct virtio_vq_info
 * @hwrng	: device specific member
 * @node	: hashtable maintaining multiple connections
 *		  from multiple guests/devices
 */
struct ipu4_virtio_be_priv {
	struct virtio_dev_info dev;
	struct virtio_vq_info vqs[IPU_VIRTIO_QUEUE_MAX];
	bool busy;
	struct mutex mlock;
	spinlock_t slock;
	/*
	 * Each VBS-K module might serve multiple connections
	 * from multiple guests/device models/VBS-Us, so better
	 * to maintain the connections in a list, and here we
	 * use hashtable as an example.
	 */
	struct hlist_node node;
};

#define RNG_MAX_HASH_BITS 4		/* MAX is 2^4 */
#define HASH_NAME vbs_hash

DECLARE_HASHTABLE(HASH_NAME, RNG_MAX_HASH_BITS);
static int ipu_vbk_hash_initialized;
static int ipu_vbk_connection_cnt;
/* function declarations */
static int handle_kick(int client_id, unsigned long *req_map,
		       void *client_priv);
static void ipu_vbk_reset(struct ipu4_virtio_be_priv *rng);
static void ipu_vbk_stop(struct ipu4_virtio_be_priv *rng);
static void ipu_vbk_flush(struct ipu4_virtio_be_priv *rng);

#ifdef RUNTIME_CTRL
static int ipu_vbk_enable_vq(struct ipu4_virtio_be_priv *rng,
			     struct virtio_vq_info *vq);
static void ipu_vbk_disable_vq(struct ipu4_virtio_be_priv *rng,
			       struct virtio_vq_info *vq);
static void ipu_vbk_stop_vq(struct ipu4_virtio_be_priv *rng,
			      struct virtio_vq_info *vq);
static void ipu_vbk_flush_vq(struct ipu4_virtio_be_priv *rng, int index);
#endif

/* hash table related functions */
static void ipu_vbk_hash_init(void)
{
	if (ipu_vbk_hash_initialized)
		return;

	hash_init(HASH_NAME);
	ipu_vbk_hash_initialized = 1;
}

static int ipu_vbk_hash_add(struct ipu4_virtio_be_priv *entry)
{
	if (!ipu_vbk_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_add(HASH_NAME, &entry->node, virtio_dev_client_id(&entry->dev));
	return 0;
}

static struct ipu4_virtio_be_priv *ipu_vbk_hash_find(int client_id)
{
	struct ipu4_virtio_be_priv *entry;
	int bkt;

	if (!ipu_vbk_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return NULL;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		if (virtio_dev_client_id(&entry->dev) == client_id)
			return entry;

	pr_err("Not found item matching client_id!\n");
	return NULL;
}

static int ipu_vbk_hash_del(int client_id)
{
	struct ipu4_virtio_be_priv *entry;
	int bkt;

	if (!ipu_vbk_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		if (virtio_dev_client_id(&entry->dev) == client_id) {
			hash_del(&entry->node);
			return 0;
		}

	pr_err("%s failed, not found matching client_id!\n",
	       __func__);
	return -1;
}

static int ipu_vbk_hash_del_all(void)
{
	struct ipu4_virtio_be_priv *entry;
	int bkt;

	if (!ipu_vbk_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		hash_del(&entry->node);

	return 0;
}

static void handle_vq_kick(int client_id, int vq_idx)
{
	struct iovec iov;
	struct ipu4_virtio_be_priv *be;
	struct virtio_vq_info *vq;
	struct ipu4_virtio_req_info *req_info = NULL;
	struct ipu4_virtio_req *req = NULL;
	int len;
	int ret;
	uint16_t idx;

	be = ipu_vbk_hash_find(client_id);
	if (be == NULL) {
		pr_err("%s: client %d not found!\n",
				__func__, client_id);
		return;
	}

	if (!be) {
		pr_err("rng is NULL! Cannot proceed!\n");
		return;
	}

	vq = &(be->vqs[vq_idx]);

	while (virtio_vq_has_descs(vq)) {
		virtio_vq_getchain(vq, &idx, &iov, 1, NULL);

		pr_debug("%s: vq index: %d vq buf index: %d req ptr: %lu\n",
						__func__, vq_idx, idx, (long unsigned)iov.iov_base);
		/* device specific operations, for example: */
		if (iov.iov_len != sizeof(struct ipu4_virtio_req)) {
			if (iov.iov_len == sizeof(int)) {
					*((int *)iov.iov_base) = 1;
					len = iov.iov_len;
					printk(KERN_NOTICE "IPU VBK handle kick from vmid:%d\n", 1);
			} else {
					len = 0;
					printk(KERN_WARNING "received request with wrong size");
					printk(KERN_WARNING "%zu != %zu\n",
							iov.iov_len,
							sizeof(struct ipu4_virtio_req));
			}

			pr_debug("vtrnd: vtrnd_notify(): %d\r\n", len);
			virtio_vq_relchain(vq, idx, len);
			continue;
		}

		req_info = ipu4_virtio_be_req_queue_get();
		if (req_info) {
			req = (struct ipu4_virtio_req *)iov.iov_base;
			req_info->request = req;
			req_info->vq_info.req_len = iov.iov_len;
			req_info->vq_info.vq_buf_idx = idx;
			req_info->vq_info.vq_idx = vq_idx;
			req_info->domid = 1;
			req_info->client_id = client_id;
			ret = intel_ipu4_virtio_msg_parse(req_info);
		} else {
			pr_err("%s: Failed to get request buffer from queue!", __func__);
			virtio_vq_relchain(vq, idx, iov.iov_len);
			continue;
		}

		if (req->stat != IPU4_REQ_PENDING) {
			virtio_vq_relchain(vq, idx, iov.iov_len);
			ipu4_virtio_be_req_queue_put(req_info);
		}
		pr_debug("%s ending request for stream %d",
			__func__, req->op[0]);
	}
	pr_debug("IPU VBK data process on VQ Done\n");
	if ((req == NULL) || (req && req->stat !=
						IPU4_REQ_PENDING))
		virtio_vq_endchains(vq, 1);
}

static int handle_kick(int client_id, unsigned long *ioreqs_map,
		       void *client_priv)
{
	int *val, i, count;
	struct ipu4_virtio_be_priv *priv;

	if (unlikely(bitmap_empty(ioreqs_map, VHM_REQUEST_MAX)))
		return -EINVAL;

	priv = container_of(client_priv, struct ipu4_virtio_be_priv, dev);
	if (priv == NULL) {
		pr_err("%s: client %d not found!\n",
				__func__, client_id);
		return -EINVAL;
	}

	val = kzalloc(priv->dev._ctx.max_vcpu * sizeof(int),
										GFP_KERNEL);

	count = virtio_vqs_index_get(&priv->dev, ioreqs_map, val,
								priv->dev._ctx.max_vcpu);

	for (i = 0; i < count; i++) {
		if (val[i] >= 0) {
			handle_vq_kick(client_id, val[i]);
		}
	}

	kfree(val);

	return 0;
}

static int ipu_vbk_open(struct inode *inode, struct file *f)
{
	struct ipu4_virtio_be_priv *priv;
	struct virtio_dev_info *dev;
	struct virtio_vq_info *vqs;
	int i;

	priv = kcalloc(1, sizeof(struct ipu4_virtio_be_priv),
		      GFP_KERNEL);

	if (priv == NULL) {
		pr_err("Failed to allocate memory for ipu4_virtio_be_priv!\n");
		return -ENOMEM;
	}

	vqs = &priv->vqs[0];

	dev = &priv->dev;

	strncpy(dev->name, "vbs_ipu", VBS_NAME_LEN);
	dev->dev_notify = handle_kick;


	for (i = 0; i < IPU_VIRTIO_QUEUE_MAX; i++) {
		vqs[i].dev = dev;
		vqs[i].vq_notify = NULL;
	}

	/* link dev and vqs */
	dev->vqs = vqs;

	virtio_dev_init(dev, vqs, IPU_VIRTIO_QUEUE_MAX);

	mutex_init(&priv->mlock);
	spin_lock_init(&priv->slock);

	f->private_data = priv;

	/* init a hash table to maintain multi-connections */
	ipu_vbk_hash_init();

	ipu4_virtio_be_req_queue_init();

	return 0;
}

static int ipu_vbk_release(struct inode *inode, struct file *f)
{
	struct ipu4_virtio_be_priv *priv = f->private_data;
	int i;

	if (!priv)
		pr_err("%s: UNLIKELY rng NULL!\n",
		       __func__);

	ipu_vbk_stop(priv);
	ipu_vbk_flush(priv);
	for (i = 0; i < IPU_VIRTIO_QUEUE_MAX; i++)
		virtio_vq_reset(&(priv->vqs[i]));

	/* device specific release */
	ipu_vbk_reset(priv);

	pr_debug("ipu_vbk_connection cnt is %d\n",
			ipu_vbk_connection_cnt);

	if (priv && ipu_vbk_connection_cnt--)
		ipu_vbk_hash_del(virtio_dev_client_id(&priv->dev));
	if (!ipu_vbk_connection_cnt) {
		pr_debug("ipu4_virtio_be_priv remove all hash entries\n");
		ipu_vbk_hash_del_all();
	}

	kfree(priv);

	ipu4_virtio_be_req_queue_free();

	pr_debug("%s done\n", __func__);
	return 0;
}

static long ipu_vbk_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	struct ipu4_virtio_be_priv *priv = f->private_data;
	void __user *argp = (void __user *)arg;
	/*u64 __user *featurep = argp;*/
	/*u64 features;*/
	int r;

	if (priv == NULL) {
		pr_err("No IPU backend private data\n");
		return -EINVAL;
	}
	switch (ioctl) {
/*
 *	case VHOST_GET_FEATURES:
 *		features = VHOST_NET_FEATURES;
 *		if (copy_to_user(featurep, &features, sizeof features))
 *			return -EFAULT;
 *		return 0;
 *	case VHOST_SET_FEATURES:
 *		if (copy_from_user(&features, featurep, sizeof features))
 *			return -EFAULT;
 *		if (features & ~VHOST_NET_FEATURES)
 *			return -EOPNOTSUPP;
 *		return vhost_net_set_features(n, features);
 */
	case VBS_SET_VQ:
		/*
		 * we handle this here because we want to register VHM client
		 * after handling VBS_K_SET_VQ request
		 */
		r = virtio_vqs_ioctl(&priv->dev, ioctl, argp);
		if (r == -ENOIOCTLCMD) {
			pr_err("VBS_K_SET_VQ: virtio_vqs_ioctl failed!\n");
			return -EFAULT;
		}
		/* Register VHM client */
		if (virtio_dev_register(&priv->dev) < 0) {
			pr_err("failed to register VHM client!\n");
			return -EFAULT;
		}
		/* Added to local hash table */
		if (ipu_vbk_hash_add(priv) < 0) {
			pr_err("failed to add to hashtable!\n");
			return -EFAULT;
		}
		/* Increment counter */
		ipu_vbk_connection_cnt++;
		return r;
	default:
		/*mutex_lock(&n->dev.mutex);*/
		r = virtio_dev_ioctl(&priv->dev, ioctl, argp);
		if (r == -ENOIOCTLCMD)
			r = virtio_vqs_ioctl(&priv->dev, ioctl, argp);
		else
			ipu_vbk_flush(priv);
		/*mutex_unlock(&n->dev.mutex);*/
		return r;
	}
}

int notify_fe(int status, struct ipu4_virtio_req_info *req_info)
{
	struct virtio_vq_info *vq;
	struct ipu4_virtio_be_priv *be;
	unsigned long flags = 0;

	pr_debug("%s: stream: %d vq idx: %d cmd: %d stat: %d",
		__func__, req_info->request->op[0],
		req_info->vq_info.vq_idx,
		req_info->request->cmd,
		status);

	be = ipu_vbk_hash_find(req_info->client_id);
	if (be == NULL) {
		pr_err("%s: client %d not found!\n",
				__func__, req_info->client_id);
		return -EINVAL;
	}

	vq = &(be->vqs[req_info->vq_info.vq_idx]);

	req_info->request->stat = status;

	spin_lock_irqsave(&be->slock, flags);
	virtio_vq_relchain(vq, req_info->vq_info.vq_buf_idx,
				req_info->vq_info.req_len);
	virtio_vq_endchains(vq, 1);
	ipu4_virtio_be_req_queue_put(req_info);
	spin_unlock_irqrestore(&be->slock, flags);

	return 0;
}

/* device specific function to cleanup itself */
static void ipu_vbk_reset(struct ipu4_virtio_be_priv *rng)
{
}

/* device specific function */
static void ipu_vbk_stop(struct ipu4_virtio_be_priv *rng)
{
	virtio_dev_deregister(&rng->dev);
}

/* device specific function */
static void ipu_vbk_flush(struct ipu4_virtio_be_priv *rng)
{
}

#ifdef RUNTIME_CTRL
/* device specific function */
static int ipu_vbk_enable_vq(struct ipu4_virtio_be_priv *rng,
			     struct virtio_vq_info *vq)
{
	return 0;
}

/* device specific function */
static void ipu_vbk_disable_vq(struct ipu4_virtio_be_priv *rng,
			       struct virtio_vq_info *vq)
{
}

/* device specific function */
static void ipu_vbk_stop_vq(struct ipu4_virtio_be_priv *rng,
			      struct virtio_vq_info *vq)
{
}

/* device specific function */
static void ipu_vbk_flush_vq(struct ipu4_virtio_be_priv *rng, int index)
{
}

/* Set feature bits in kernel side device */
static int ipu_vbk_set_features(struct ipu4_virtio_be_priv *rng, u64 features)
{
	return 0;
}
#endif

static const struct file_operations vbs_fops = {
	.owner          = THIS_MODULE,
	.release        = ipu_vbk_release,
	.unlocked_ioctl = ipu_vbk_ioctl,
	.open           = ipu_vbk_open,
	.llseek		= noop_llseek,
};

static struct miscdevice vbs_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vbs_ipu",
	.fops = &vbs_fops,
};

static int ipu_vbk_init(void)
{
	return misc_register(&vbs_misc);
}
module_init(ipu_vbk_init);

static void ipu_vbk_exit(void)
{
	misc_deregister(&vbs_misc);
}
module_exit(ipu_vbk_exit);

MODULE_VERSION("0.1");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("IPU4 virtio driver");
MODULE_LICENSE("Dual BSD/GPL");
