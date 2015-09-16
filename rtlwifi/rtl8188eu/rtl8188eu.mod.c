#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xcc606cf9, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xf5c1f262, __VMLINUX_SYMBOL_STR(rtl_cam_delete_one_entry) },
	{ 0xe3ad6d8c, __VMLINUX_SYMBOL_STR(rtl_fw_cb) },
	{ 0xd26f74a2, __VMLINUX_SYMBOL_STR(rtl_cam_get_free_entry) },
	{ 0x722faf0f, __VMLINUX_SYMBOL_STR(rtl_usb_disconnect) },
	{ 0x6bf1c17f, __VMLINUX_SYMBOL_STR(pv_lock_ops) },
	{ 0xac1476ac, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xdb80a56e, __VMLINUX_SYMBOL_STR(rtl_dm_diginit) },
	{ 0x6c4a0593, __VMLINUX_SYMBOL_STR(rtl_ps_disable_nic) },
	{ 0xd2deac45, __VMLINUX_SYMBOL_STR(rtl_hal_pwrseqcmdparsing) },
	{ 0xd9d3bcd3, __VMLINUX_SYMBOL_STR(_raw_spin_lock_bh) },
	{ 0x124e41d4, __VMLINUX_SYMBOL_STR(rtl_get_tcb_desc) },
	{ 0x265b74ee, __VMLINUX_SYMBOL_STR(rtl_efuse_shadow_map_update) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0x79188a02, __VMLINUX_SYMBOL_STR(rtl_query_rxpwrpercentage) },
	{ 0xb5732e1c, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0x98d76d9b, __VMLINUX_SYMBOL_STR(rtl_cam_add_one_entry) },
	{ 0x999e8297, __VMLINUX_SYMBOL_STR(vfree) },
	{ 0x4629334c, __VMLINUX_SYMBOL_STR(__preempt_count) },
	{ 0x207fbb70, __VMLINUX_SYMBOL_STR(rtl_evm_db_to_percentage) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0xd5540b3, __VMLINUX_SYMBOL_STR(rtl_phy_scan_operation_backup) },
	{ 0x78897bde, __VMLINUX_SYMBOL_STR(rtl_usb_probe) },
	{ 0x7794cbee, __VMLINUX_SYMBOL_STR(__netdev_alloc_skb) },
	{ 0xb4132fbd, __VMLINUX_SYMBOL_STR(rtl_ps_enable_nic) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0xd10ca20c, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x37befc70, __VMLINUX_SYMBOL_STR(jiffies_to_msecs) },
	{ 0xf2ce6be5, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x449ad0a7, __VMLINUX_SYMBOL_STR(memcmp) },
	{ 0xb12702a3, __VMLINUX_SYMBOL_STR(skb_push) },
	{ 0x16e5c2a, __VMLINUX_SYMBOL_STR(mod_timer) },
	{ 0xa4c8c56b, __VMLINUX_SYMBOL_STR(rtl_signal_scale_mapping) },
	{ 0xaef9e867, __VMLINUX_SYMBOL_STR(request_firmware_nowait) },
	{ 0xfa3bdaee, __VMLINUX_SYMBOL_STR(rtlwifi_rate_mapping) },
	{ 0x167c5967, __VMLINUX_SYMBOL_STR(print_hex_dump) },
	{ 0x40a9b349, __VMLINUX_SYMBOL_STR(vzalloc) },
	{ 0x78764f4e, __VMLINUX_SYMBOL_STR(pv_irq_ops) },
	{ 0xbba70a2d, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_bh) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x96a9d9dd, __VMLINUX_SYMBOL_STR(ieee80211_find_sta) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xe259ae9e, __VMLINUX_SYMBOL_STR(_raw_spin_lock) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0xa5ca39f, __VMLINUX_SYMBOL_STR(rtl_cam_mark_invalid) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x66d2b955, __VMLINUX_SYMBOL_STR(rtl_cam_del_entry) },
	{ 0xf0f32b2f, __VMLINUX_SYMBOL_STR(rtl_cam_empty_entry) },
	{ 0x9fe7f65d, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x162c1c6b, __VMLINUX_SYMBOL_STR(skb_dequeue) },
	{ 0xf30eba96, __VMLINUX_SYMBOL_STR(rtl_cam_reset_all_entry) },
	{ 0xd1f844fa, __VMLINUX_SYMBOL_STR(rtl_process_phyinfo) },
	{ 0xaff69954, __VMLINUX_SYMBOL_STR(skb_put) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=rtlwifi,rtl_usb,mac80211";

MODULE_ALIAS("usb:v0BDAp8179d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BDAp0179d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v056Ep4008d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v07B8p8179d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2001p330Fd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2001p3310d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2001p3311d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0DF6p0076d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "5641A575B4DFC255EC688D8");
