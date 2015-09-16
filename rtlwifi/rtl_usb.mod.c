#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xcc606cf9, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xdcb5c57c, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x2f024c47, __VMLINUX_SYMBOL_STR(rtl_deinit_deferred_work) },
	{ 0xc0f52dc8, __VMLINUX_SYMBOL_STR(rtl_deinit_core) },
	{ 0xce83053e, __VMLINUX_SYMBOL_STR(usb_get_from_anchor) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x448eac3e, __VMLINUX_SYMBOL_STR(kmemdup) },
	{ 0xe7eac2a, __VMLINUX_SYMBOL_STR(ieee80211_unregister_hw) },
	{ 0x6de7cd26, __VMLINUX_SYMBOL_STR(__dev_kfree_skb_any) },
	{ 0x4629334c, __VMLINUX_SYMBOL_STR(__preempt_count) },
	{ 0x72f25a7e, __VMLINUX_SYMBOL_STR(read_efuse_byte) },
	{ 0x171ab421, __VMLINUX_SYMBOL_STR(usb_unanchor_urb) },
	{ 0x7794cbee, __VMLINUX_SYMBOL_STR(__netdev_alloc_skb) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x64ab0e98, __VMLINUX_SYMBOL_STR(wait_for_completion) },
	{ 0x24d7566, __VMLINUX_SYMBOL_STR(skb_queue_purge) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0x22d8eef4, __VMLINUX_SYMBOL_STR(ieee80211_alloc_hw_nm) },
	{ 0x1a14fabb, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x52b382a4, __VMLINUX_SYMBOL_STR(rtl_init_core) },
	{ 0xfaef0ed, __VMLINUX_SYMBOL_STR(__tasklet_schedule) },
	{ 0x4d712993, __VMLINUX_SYMBOL_STR(rtl_action_proc) },
	{ 0x4315ca7c, __VMLINUX_SYMBOL_STR(rtl_dbgp_flag_init) },
	{ 0x5aae93f0, __VMLINUX_SYMBOL_STR(usb_control_msg) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0x8374a1d9, __VMLINUX_SYMBOL_STR(ieee80211_rx) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0x6b4de810, __VMLINUX_SYMBOL_STR(rtl_lps_change_work_callback) },
	{ 0x2512a79a, __VMLINUX_SYMBOL_STR(rtl_init_rx_config) },
	{ 0x6a251a0c, __VMLINUX_SYMBOL_STR(skb_pull) },
	{ 0x5fa9f650, __VMLINUX_SYMBOL_STR(usb_free_coherent) },
	{ 0x42160169, __VMLINUX_SYMBOL_STR(flush_workqueue) },
	{ 0x82072614, __VMLINUX_SYMBOL_STR(tasklet_kill) },
	{ 0xb8b6a27b, __VMLINUX_SYMBOL_STR(skb_queue_tail) },
	{ 0x36cddde8, __VMLINUX_SYMBOL_STR(usb_submit_urb) },
	{ 0x8a799a80, __VMLINUX_SYMBOL_STR(usb_get_dev) },
	{ 0x4395953e, __VMLINUX_SYMBOL_STR(usb_kill_anchored_urbs) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x28fd33ef, __VMLINUX_SYMBOL_STR(usb_put_dev) },
	{ 0x255fc732, __VMLINUX_SYMBOL_STR(ieee80211_tx_status_irqsafe) },
	{ 0x2b2eebb2, __VMLINUX_SYMBOL_STR(rtl_ops) },
	{ 0xbc6cee4b, __VMLINUX_SYMBOL_STR(kfree_skb) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xb72e3a71, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0x3512ce6, __VMLINUX_SYMBOL_STR(ieee80211_register_hw) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x26d84afb, __VMLINUX_SYMBOL_STR(rtl_ips_nic_on) },
	{ 0xdfd87c23, __VMLINUX_SYMBOL_STR(ieee80211_free_hw) },
	{ 0x162c1c6b, __VMLINUX_SYMBOL_STR(skb_dequeue) },
	{ 0xb2d5a552, __VMLINUX_SYMBOL_STR(complete) },
	{ 0x5e1fd5ca, __VMLINUX_SYMBOL_STR(usb_alloc_coherent) },
	{ 0xaff69954, __VMLINUX_SYMBOL_STR(skb_put) },
	{ 0x94ac9a6d, __VMLINUX_SYMBOL_STR(usb_free_urb) },
	{ 0x6faff4, __VMLINUX_SYMBOL_STR(rtl_beacon_statistic) },
	{ 0x984a141c, __VMLINUX_SYMBOL_STR(usb_anchor_urb) },
	{ 0xed8016c7, __VMLINUX_SYMBOL_STR(usb_alloc_urb) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=rtlwifi,mac80211";


MODULE_INFO(srcversion, "08BF01507EAE221F08A1A27");
