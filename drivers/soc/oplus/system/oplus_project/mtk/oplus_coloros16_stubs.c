// SPDX-License-Identifier: GPL-2.0
/*
 * ColorOS 16 Compatibility Stubs
 *
 * Provides missing /proc and sysfs interfaces required by ColorOS 16 (Android 16)
 * to prevent polling storms, scheduling failures, and thermal management errors.
 *
 * Copyright (C) 2026 xCaptaiN09
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/thermal.h>

#define STUB_LOG(fmt, ...) pr_info_once("[ColorOS16-Stub] " fmt, ##__VA_ARGS__)

/* ========================================
 * Generic Stub Operations
 * ======================================== */

static int generic_stub_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0\n");
	return 0;
}

static int generic_stub_open(struct inode *inode, struct file *file)
{
	return single_open(file, generic_stub_show, NULL);
}

static ssize_t generic_stub_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	STUB_LOG("Ignored write to %s\n", file->f_path.dentry->d_name.name);
	return count;
}

static const struct file_operations generic_stub_fops = {
	.owner   = THIS_MODULE,
	.open    = generic_stub_open,
	.read    = seq_read,
	.write   = generic_stub_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

/* Read-only stub that returns empty/zero */
static int ro_zero_stub_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0\n");
	return 0;
}

static int ro_zero_stub_open(struct inode *inode, struct file *file)
{
	return single_open(file, ro_zero_stub_show, NULL);
}

static const struct file_operations ro_zero_stub_fops = {
	.owner   = THIS_MODULE,
	.open    = ro_zero_stub_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

/* ========================================
 * QoS Scheduler Stubs
 * ======================================== */

static void create_qos_stubs(void)
{
	struct proc_dir_entry *qos_dir;

	/* /proc/oplus_qos */
	qos_dir = proc_mkdir("oplus_qos", NULL);
	if (qos_dir) {
		proc_create("qos_enable", 0666, qos_dir, &generic_stub_fops);
		proc_create("qos_scene", 0666, qos_dir, &generic_stub_fops);
		proc_create("qos_strategy", 0666, qos_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/oplus_qos stubs\n");
	}

	/* /proc/oplus_qos_monitor */
	qos_dir = proc_mkdir("oplus_qos_monitor", NULL);
	if (qos_dir) {
		proc_create("monitor_config", 0666, qos_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/oplus_qos_monitor stubs\n");
	}
}

/* ========================================
 * UserAwareMgr / Thread Scheduling Stubs
 * ======================================== */

static int useraware_stub_show(struct seq_file *m, void *v)
{
	/* Return a valid-looking config to satisfy UserAwareMgr */
	seq_printf(m, "1 10 1 100\n");
	return 0;
}

static int useraware_stub_open(struct inode *inode, struct file *file)
{
	return single_open(file, useraware_stub_show, NULL);
}

static const struct file_operations useraware_stub_fops = {
	.owner   = THIS_MODULE,
	.open    = useraware_stub_open,
	.read    = seq_read,
	.write   = generic_stub_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void create_useraware_stubs(void)
{
	struct proc_dir_entry *ua_dir;

	/* /proc/oplus_useraware */
	ua_dir = proc_mkdir("oplus_useraware", NULL);
	if (ua_dir) {
		proc_create("useraware_config", 0666, ua_dir, &useraware_stub_fops);
		proc_create("thread_group", 0666, ua_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/oplus_useraware stubs\n");
	}
}

/* ========================================
 * Thermal Management Stubs
 * ======================================== */

static int thermal_cpu_stub_show(struct seq_file *m, void *v)
{
	/* Return valid thermal config format */
	seq_printf(m, "4 800000 1200000 1500000 1800000\n");
	return 0;
}

static int thermal_cpu_stub_open(struct inode *inode, struct file *file)
{
	return single_open(file, thermal_cpu_stub_show, NULL);
}

static const struct file_operations thermal_cpu_stub_fops = {
	.owner   = THIS_MODULE,
	.open    = thermal_cpu_stub_open,
	.read    = seq_read,
	.write   = generic_stub_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int thermal_gpu_stub_show(struct seq_file *m, void *v)
{
	seq_printf(m, "4 300000000 600000000 900000000\n");
	return 0;
}

static int thermal_gpu_stub_open(struct inode *inode, struct file *file)
{
	return single_open(file, thermal_gpu_stub_show, NULL);
}

static const struct file_operations thermal_gpu_stub_fops = {
	.owner   = THIS_MODULE,
	.open    = thermal_gpu_stub_open,
	.read    = seq_read,
	.write   = generic_stub_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void create_thermal_stubs(void)
{
	struct proc_dir_entry *thermal_dir;

	/* /proc/thermal */
	thermal_dir = proc_mkdir("thermal", NULL);
	if (thermal_dir) {
		proc_create("cpu_feature_config", 0666, thermal_dir, &thermal_cpu_stub_fops);
		proc_create("gpu_feature_config", 0666, thermal_dir, &thermal_gpu_stub_fops);
		proc_create("thermal_mode", 0666, thermal_dir, &generic_stub_fops);
		proc_create("thermal_policy", 0666, thermal_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/thermal stubs\n");
	}

	/* /sys/class/thermal/thermal_message */
	/* Note: This would need a class_device, skip for now - HAL handles missing gracefully */
}

/* ========================================
 * Audio Sched Boost Stub
 * ======================================== */

static void create_audio_sched_stubs(void)
{
	struct proc_dir_entry *audio_dir;

	/* /proc/audio_sched_boost */
	audio_dir = proc_mkdir("audio_sched", NULL);
	if (audio_dir) {
		proc_create("audio_boost_enable", 0666, audio_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/audio_sched stubs\n");
	}
}

/* ========================================
 * Cgroupv2 CPU Subtree Control
 * ======================================== */

static int cgroup_cpu_subtree_show(struct seq_file *m, void *v)
{
	/* Return enabled cgroup subtree control */
	seq_printf(m, "cpuset cpu io memory pids\n");
	return 0;
}

static int cgroup_cpu_subtree_open(struct inode *inode, struct file *file)
{
	return single_open(file, cgroup_cpu_subtree_show, NULL);
}

static const struct file_operations cgroup_cpu_subtree_fops = {
	.owner   = THIS_MODULE,
	.open    = cgroup_cpu_subtree_open,
	.read    = seq_read,
	.write   = generic_stub_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void create_cgroup_stubs(void)
{
	struct proc_dir_entry *cgroup_dir;

	/* /proc/cgroup_info */
	cgroup_dir = proc_mkdir("cgroup_info", NULL);
	if (cgroup_dir) {
		proc_create("cpu_subtree_control", 0666, cgroup_dir, &cgroup_cpu_subtree_fops);
		proc_create("cgroup_version", 0444, cgroup_dir, &ro_zero_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/cgroup_info stubs\n");
	}
}

/* ========================================
 * Display VRR/TE Additional Stubs
 * ======================================== */

static void create_display_stubs(void)
{
	struct proc_dir_entry *disp_dir;

	/* /proc/oplus_display */
	disp_dir = proc_mkdir("oplus_display", NULL);
	if (disp_dir) {
		proc_create("vrr_enable", 0666, disp_dir, &ro_zero_stub_fops);
		proc_create("te_config", 0666, disp_dir, &generic_stub_fops);
		proc_create("panel_fps", 0666, disp_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/oplus_display stubs\n");
	}
}

/* ========================================
 * MBrain / App Freeze Stubs
 * ======================================== */

static void create_mbrain_stubs(void)
{
	struct proc_dir_entry *mbrain_dir;

	/* /proc/mbrain */
	mbrain_dir = proc_mkdir("mbrain", NULL);
	if (mbrain_dir) {
		proc_create("freeze_config", 0666, mbrain_dir, &generic_stub_fops);
		proc_create("lifecycle_config", 0666, mbrain_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/mbrain stubs\n");
	}
}

/* ========================================
 * Houston / Perf Lock Stubs
 * ======================================== */

static void create_houston_stubs(void)
{
	struct proc_dir_entry *houston_dir;

	/* /proc/houston */
	houston_dir = proc_mkdir("houston", NULL);
	if (houston_dir) {
		proc_create("houston_enable", 0666, houston_dir, &generic_stub_fops);
		proc_create("perf_lock", 0666, houston_dir, &generic_stub_fops);
		pr_info("[ColorOS16-Stub] Created /proc/houston stubs\n");
	}
}

/* ========================================
 * Init Function
 * ======================================== */

static int __init oplus_coloros16_stubs_init(void)
{
	pr_info("[ColorOS16-Stub] Initializing ColorOS 16 compatibility stubs\n");

	create_qos_stubs();
	create_useraware_stubs();
	create_thermal_stubs();
	create_audio_sched_stubs();
	create_cgroup_stubs();
	create_display_stubs();
	create_mbrain_stubs();
	create_houston_stubs();

	pr_info("[ColorOS16-Stub] ColorOS 16 compatibility stubs initialized\n");
	return 0;
}

/* Use late_initcall to ensure stubs are available after all modules load */
late_initcall(oplus_coloros16_stubs_init);

MODULE_DESCRIPTION("ColorOS 16 Compatibility Stubs");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("xCaptaiN09");
