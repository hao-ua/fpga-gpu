// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * fpgagpu: the DRM driver of the FPGA GPU, two FPGA boards that a PC drives over Ethernet.
 *
 * The boards are on no bus the kernel probes, so the module registers a platform device "fpgagpu" and binds its own
 * platform driver to it. Probing opens the UDP transport (fpgagpu_net.c), greets board A, reads both boards' STATUS
 * and registers the DRM device: a render node with the GET_PARAM and GET_STATUS ioctls (include/uapi/drm/fpgagpu_drm.h).
 * While board A does not answer (the boards off, the network not up yet at boot) the module stays loaded and greets
 * it again every few seconds; the DRM device appears once it answers.
 *   modprobe fpgagpu ip=192.168.1.10          board A on this host's network
 *   modprobe fpgagpu ip=127.0.0.1             through host/linux/tools/g2_relay.py from a remote host
 */
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_fbdev_shmem.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
#include <drm/drm_fbdev_shmem.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#include <drm/drm_fbdev_generic.h>
#else
#include <drm/drm_fb_helper.h>
#endif

#include "fpgagpu.h"
#include "fpgagpu_drm.h"

static char *ip = "192.168.1.10";
module_param(ip, charp, 0444);
MODULE_PARM_DESC(ip, "board A's IPv4 address (127.0.0.1 through host/linux/tools/g2_relay.py)");

static ushort port = 5000;
module_param(port, ushort, 0444);
MODULE_PARM_DESC(port, "board A's UDP port");

#define STATUS_TIMEOUT_MS	1500		/* three requests; enough for a round trip of a few hundred ms */

/* ------------------------------------------------------------------ ioctls */

static int fpgagpu_status_word(struct fpgagpu_device *fdev, unsigned int idx, u64 *value)
{
	u32 *w = kmalloc_array(FG_STATUS_WORDS, sizeof(u32), GFP_KERNEL);
	u32 got = 0;
	int r;

	if (!w)
		return -ENOMEM;
	r = fg_net_status(&fdev->net, w, FG_STATUS_WORDS, &got, NULL, STATUS_TIMEOUT_MS);
	if (!r && idx >= got)
		r = -ENODATA;
	if (!r)
		*value = w[idx];
	kfree(w);
	return r;
}

static int fpgagpu_ioctl_get_param(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_get_param *a = data;

	if (a->pad)
		return -EINVAL;
	switch (a->param) {
	case FPGAGPU_PARAM_STATUS_LAYOUT:
		return fpgagpu_status_word(fdev, FG_ST_LAYOUT, &a->value);
	case FPGAGPU_PARAM_A_BUILD:
		return fpgagpu_status_word(fdev, FG_ST_BUILD, &a->value);
	case FPGAGPU_PARAM_A_FLAGS:
		return fpgagpu_status_word(fdev, FG_ST_FLAGS, &a->value);
	case FPGAGPU_PARAM_B_FLAGS:
		return fpgagpu_status_word(fdev, FG_ST_B_FLAGS, &a->value);
	case FPGAGPU_PARAM_LINK_FLAGS:
		return fpgagpu_status_word(fdev, FG_ST_LINK_FLAGS, &a->value);
	case FPGAGPU_PARAM_RTT_US:
		a->value = fg_net_rtt_us(&fdev->net);
		return 0;
	default:
		return -EINVAL;
	}
}

static int fpgagpu_ioctl_get_status(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_get_status *a = data;
	u32 room = min_t(u32, a->count, FG_STATUS_WORDS);
	u32 *w, got = 0;
	u64 serial = 0;
	int r;

	if (a->flags)
		return -EINVAL;
	w = kmalloc_array(FG_STATUS_WORDS, sizeof(u32), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	r = fg_net_status(&fdev->net, w, room, &got, &serial, STATUS_TIMEOUT_MS);
	if (!r && got && copy_to_user(u64_to_user_ptr(a->words), w, got * sizeof(u32)))
		r = -EFAULT;
	a->count = r ? 0 : got;
	a->serial = serial;
	kfree(w);
	return r;
}

static const struct drm_ioctl_desc fpgagpu_ioctls[] = {
	DRM_IOCTL_DEF_DRV(FPGAGPU_GET_PARAM, fpgagpu_ioctl_get_param, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_GET_STATUS, fpgagpu_ioctl_get_status, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_BO_CREATE, fpgagpu_ioctl_bo_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_SUBMIT, fpgagpu_ioctl_submit, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_WAIT, fpgagpu_ioctl_wait, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_READ, fpgagpu_ioctl_read, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_BO_INFO, fpgagpu_ioctl_bo_info, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(FPGAGPU_PASS, fpgagpu_ioctl_pass, DRM_RENDER_ALLOW),
};

struct workqueue_struct *fg_wq;

DEFINE_DRM_GEM_FOPS(fpgagpu_fops);

static const struct drm_driver fpgagpu_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_RENDER | DRIVER_MODESET | DRIVER_ATOMIC,
	DRM_GEM_SHMEM_DRIVER_OPS,			/* dumb buffers and the console: shmem objects in host memory */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	DRM_FBDEV_SHMEM_DRIVER_OPS,
#endif
	.open			= fpgagpu_open,
	.postclose		= fpgagpu_postclose,	/* a pass the file held is given back */
	.ioctls			= fpgagpu_ioctls,
	.num_ioctls		= ARRAY_SIZE(fpgagpu_ioctls),
	.fops			= &fpgagpu_fops,
	.name			= "fpgagpu",
	.desc			= "FPGA GPU over Ethernet",
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 14, 0)
	.date			= "20261115",
#endif
	.major			= 0,
	.minor			= 1,
};

/* the Linux console on board B's monitor (fbdev emulation on shmem buffers) */
static void fpgagpu_fbdev_setup(struct drm_device *drm)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	drm_client_setup(drm, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
	drm_fbdev_shmem_setup(drm, 32);
#else
	drm_fbdev_generic_setup(drm, 32);
#endif
}

/* ------------------------------------------------------------------ the platform device */

#define ATTACH_RETRY_S		5		/* while board A does not answer: HELLO again this often */
#define ATTACH_HELLO_MS		1500		/* each try: three HELLOs */
#define WATCH_MS		1000		/* a STATUS request this often: a board's restart shows in the reply */

/*
 * A board restarted while the module was loaded (a power cut, a new bitstream): its tables and memory are empty, and
 * board A expects a new session. The transport stops sending when a STATUS reply shows it (fpgagpu_net.c); here, with
 * the command stream held: a new session with board A if it was the one; the descriptor of every buffer object on the
 * restarted boards again (their contents are gone); the display's mode again if board B restarted. Every file then
 * learns at its next pass that what it kept in board memory is lost (PASS's state_lost 2). After board B's restart
 * the console redraws on the hotplug event, and a change uevent with FPGAGPU_RESET=1 lets udev restart a display
 * server, whose images are gone. false: not done, to be tried again
 */
static bool fpgagpu_recover(struct fpgagpu_device *fdev, unsigned int lost, unsigned int restarts)
{
	struct device *dev = fdev->drm.dev;
	unsigned int done = 0, failed = 0;
	int r;

	dev_warn(dev, "board%s restarted: recovering\n",
		 lost == (FG_LOST_A | FG_LOST_B) ? "s A and B" : (lost & FG_LOST_A) ? " A" : " B");
	mutex_lock(&fdev->submit_lock);
	r = fg_net_recover(&fdev->net, lost, restarts, ATTACH_HELLO_MS);
	if (r) {
		mutex_unlock(&fdev->submit_lock);
		dev_warn(dev, "no new session yet (%d)\n", r);
		return false;
	}
	fg_bo_recreate(fdev, lost, &done, &failed);
	fdev->frame_owner = NULL;			/* the frame a file had open went with the boards */
	if (lost & FG_LOST_B)
		fg_kms_board_lost(fdev);
	mutex_unlock(&fdev->submit_lock);
	mutex_lock(&fdev->pass_lock);
	fdev->reset_gen++;
	mutex_unlock(&fdev->pass_lock);
	dev_warn(dev, "recovered: %s%u buffer objects described again%s, their contents lost\n",
		 (lost & FG_LOST_A) ? "a new session, " : "", done, failed ? " (some failed)" : "");
	if (lost & FG_LOST_B) {
		char *envp[] = { "FPGAGPU_RESET=1", NULL };

		drm_kms_helper_hotplug_event(&fdev->drm);
		kobject_uevent_env(&fdev->drm.primary->kdev->kobj, KOBJ_CHANGE, envp);
	}
	return true;
}

static void fpgagpu_watch_work(struct work_struct *work)
{
	struct fpgagpu_device *fdev = container_of(to_delayed_work(work), struct fpgagpu_device, watch_work);
	unsigned int lost, restarts;
	int idx;

	if (!drm_dev_enter(&fdev->drm, &idx))
		return;					/* unplugged: the module is going */
	lost = fg_net_lost(&fdev->net, &restarts);
	if (lost)
		fpgagpu_recover(fdev, lost, restarts);
	else
		fg_net_send_unseq(&fdev->net, FG_T_STATUS_REQ, NULL, 0);
	drm_dev_exit(idx);
	queue_delayed_work(fg_wq, &fdev->watch_work, msecs_to_jiffies(WATCH_MS));
}

/* greets board A; once it answers: the display and the DRM device (render node, card, console). -EAGAIN: no answer
 * (the boards are off, or the network is not up yet) */
static int fpgagpu_attach(struct fpgagpu_device *fdev)
{
	struct device *dev = fdev->drm.dev;
	u32 *w;
	u32 got = 0;
	int r;

	r = fg_net_hello(&fdev->net, ATTACH_HELLO_MS);
	if (r)
		return -EAGAIN;
	w = kmalloc_array(FG_STATUS_WORDS, sizeof(u32), GFP_KERNEL);
	if (w && !fg_net_status(&fdev->net, w, FG_STATUS_WORDS, &got, NULL, STATUS_TIMEOUT_MS) &&
	    got > FG_ST_LINK_FLAGS)
		dev_info(dev, "board A build %u, STATUS layout %u, flags %08x; board B flags %08x; link %s, B %s; round trip %u us\n",
			 w[FG_ST_BUILD], w[FG_ST_LAYOUT], w[FG_ST_FLAGS], w[FG_ST_B_FLAGS],
			 FG_LINK_TRAINED(w[FG_ST_LINK_FLAGS]) ? "trained" : "NOT trained",
			 FG_LINK_B_ALIVE(w[FG_ST_LINK_FLAGS]) ? "alive" : "NOT alive", fg_net_rtt_us(&fdev->net));
	else
		dev_warn(dev, "no STATUS from %s:%u\n", ip, port);
	kfree(w);

	r = fg_kms_init(fdev);
	if (r)
		goto fail;
	r = drm_dev_register(&fdev->drm, 0);
	if (r)
		goto fail;
	fpgagpu_fbdev_setup(&fdev->drm);
	fdev->attached = true;
	queue_delayed_work(fg_wq, &fdev->watch_work, msecs_to_jiffies(WATCH_MS));
	return 0;
fail:
	fg_kms_fini(fdev);
	return r;
}

static void fpgagpu_attach_work(struct work_struct *work)
{
	struct fpgagpu_device *fdev = container_of(to_delayed_work(work), struct fpgagpu_device, attach_work);
	int r = fpgagpu_attach(fdev);

	fdev->attach_tries++;
	if (r == -EAGAIN) {
		queue_delayed_work(fg_wq, &fdev->attach_work, ATTACH_RETRY_S * HZ);
		return;
	}
	if (r)
		dev_err(fdev->drm.dev, "board A answered after %u tries, but the device could not be set up (%d)\n",
			fdev->attach_tries, r);
	else
		dev_info(fdev->drm.dev, "board A answered after %u tries\n", fdev->attach_tries);
}

static int fpgagpu_probe(struct platform_device *pdev)
{
	struct fpgagpu_device *fdev;
	int r;

	fdev = devm_drm_dev_alloc(&pdev->dev, &fpgagpu_drm_driver, struct fpgagpu_device, drm);
	if (IS_ERR(fdev))
		return PTR_ERR(fdev);
	platform_set_drvdata(pdev, fdev);
	mutex_init(&fdev->submit_lock);
	mutex_init(&fdev->pass_lock);
	init_waitqueue_head(&fdev->pass_wq);
	spin_lock_init(&fdev->free_lock);
	INIT_LIST_HEAD(&fdev->bo_free);
	INIT_WORK(&fdev->free_work, fg_bo_free_work);
	INIT_DELAYED_WORK(&fdev->attach_work, fpgagpu_attach_work);
	INIT_DELAYED_WORK(&fdev->watch_work, fpgagpu_watch_work);
	mutex_init(&fdev->bo_lock);
	INIT_LIST_HEAD(&fdev->bo_list);
	r = fg_mem_init(fdev);
	if (r)
		return r;

	r = fg_net_init(&fdev->net, &pdev->dev, ip, port);
	if (r)
		return r;
	/* the module may load before the network is up (at boot) or with the boards off: it keeps trying, and the DRM
	 * device appears once board A answers */
	r = fpgagpu_attach(fdev);
	if (r == -EAGAIN) {
		dev_info(&pdev->dev, "board A at %s:%u does not answer; trying again every %d s\n", ip, port,
			 ATTACH_RETRY_S);
		fdev->attach_tries = 1;
		queue_delayed_work(fg_wq, &fdev->attach_work, ATTACH_RETRY_S * HZ);
		return 0;
	}
	if (r)
		fg_net_fini(&fdev->net);
	return r;
}

static void fpgagpu_remove_device(struct platform_device *pdev)
{
	struct fpgagpu_device *fdev = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&fdev->attach_work);
	if (fdev->attached) {
		drm_dev_unplug(&fdev->drm);
		cancel_delayed_work_sync(&fdev->watch_work);	/* it does not queue itself again once unplugged */
		drm_atomic_helper_shutdown(&fdev->drm);
		fg_kms_fini(fdev);
	}
	fg_net_fini(&fdev->net);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
static void fpgagpu_remove(struct platform_device *pdev)
{
	fpgagpu_remove_device(pdev);
}
#else
static int fpgagpu_remove(struct platform_device *pdev)
{
	fpgagpu_remove_device(pdev);
	return 0;
}
#endif

static struct platform_driver fpgagpu_platform_driver = {
	.probe	= fpgagpu_probe,
	.remove	= fpgagpu_remove,
	.driver	= {
		.name = "fpgagpu",
	},
};

static struct platform_device *fpgagpu_pdev;

/* a test of the recovery without restarting a board:
 *   echo 1 > /sys/module/fpgagpu/parameters/restarted      as if board A had restarted (2: board B, 3: both)
 * The boards still hold everything, so they refuse the descriptors sent again (error events in the log) */
static int fpgagpu_set_restarted(const char *val, const struct kernel_param *kp)
{
	struct fpgagpu_device *fdev = fpgagpu_pdev ? platform_get_drvdata(fpgagpu_pdev) : NULL;
	unsigned int v;
	int r = kstrtouint(val, 0, &v);

	if (r)
		return r;
	if (!v || v > (FG_LOST_A | FG_LOST_B))
		return -EINVAL;
	if (!fdev || !fdev->attached)
		return -ENODEV;
	fg_net_set_lost(&fdev->net, v);
	return 0;
}

static const struct kernel_param_ops fpgagpu_restarted_ops = {
	.set = fpgagpu_set_restarted,
};
module_param_cb(restarted, &fpgagpu_restarted_ops, NULL, 0200);
MODULE_PARM_DESC(restarted, "test: recover as if board A (1), board B (2) or both (3) had restarted");

static int __init fpgagpu_init(void)
{
	int r;

	fg_wq = alloc_workqueue("fpgagpu", 0, 0);
	if (!fg_wq)
		return -ENOMEM;
	r = platform_driver_register(&fpgagpu_platform_driver);
	if (r)
		goto fail_wq;
	fpgagpu_pdev = platform_device_register_simple("fpgagpu", -1, NULL, 0);
	if (IS_ERR(fpgagpu_pdev)) {
		r = PTR_ERR(fpgagpu_pdev);
		goto fail_drv;
	}
	return 0;
fail_drv:
	platform_driver_unregister(&fpgagpu_platform_driver);
fail_wq:
	destroy_workqueue(fg_wq);
	return r;
}

static void __exit fpgagpu_exit(void)
{
	struct platform_device *pdev = fpgagpu_pdev;

	fpgagpu_pdev = NULL;				/* the test parameter finds no device from here on */
	platform_device_unregister(pdev);
	platform_driver_unregister(&fpgagpu_platform_driver);
	destroy_workqueue(fg_wq);			/* runs the frees still queued */
}

module_init(fpgagpu_init);
module_exit(fpgagpu_exit);

MODULE_DESCRIPTION("DRM driver for the FPGA GPU over Ethernet");
MODULE_LICENSE("Dual MIT/GPL");
