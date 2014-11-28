#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
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
	{ 0xbdacf18c, "module_layout" },
	{ 0xa90c928a, "param_ops_int" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0xc990736a, "cdev_del" },
	{ 0x37a0cba, "kfree" },
	{ 0xf20dabd8, "free_irq" },
	{ 0x4b7f219c, "cdev_add" },
	{ 0x33b3a398, "cdev_alloc" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xfda85a7d, "request_threaded_irq" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xa24d2473, "davinci_iounmap" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x701d0ebd, "snprintf" },
	{ 0x98082893, "__copy_to_user" },
	{ 0x8893fa5d, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x75a17bed, "prepare_to_wait" },
	{ 0x5f754e5a, "memset" },
	{ 0xb9e52429, "__wake_up" },
	{ 0x43b0c9c3, "preempt_schedule" },
	{ 0x9ad0d04a, "davinci_ioremap" },
	{ 0xea147363, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

