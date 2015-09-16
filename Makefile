# fallback to the current kernel source
KSRC ?= /lib/modules/$(shell uname -r)/build

KMOD_SRC ?= $(CURDIR)/rtlwifi

KMOD_OPTIONS = CONFIG_RTL_CARDS=y
KMOD_OPTIONS += CONFIG_RTLWIFI=m
KMOD_OPTIONS += CONFIG_RTLWIFI_DEBUG=y
KMOD_OPTIONS += CONFIG_RTLWIFI_DEBUGFS=y
KMOD_OPTIONS += CONFIG_RTLWIFI_USB=m
KMOD_OPTIONS += CONFIG_RTL8188EU=m

# Don't build any of the other drivers
KMOD_OPTIONS += CONFIG_RTL8192C_COMMON=n CONFIG_RTL8192CE=n CONFIG_RTL8192CU=n CONFIG_RTL8192SE=n CONFIG_RTL8192DE=n CONFIG_RTL8723AE=n CONFIG_RTL8723BE=n CONFIG_RTL8188EE=n CONFIG_RTLBTCOEXIST=n CONFIG_RTL8723_COMMON=n CONFIG_RTL8821AE=n CONFIG_RTL8192EE=n CONFIG_RTLWIFI_PCI=n

EXTRA_CFLAGS += -DDEBUG -DCONFIG_RTLWIFI_DEBUGFS=m

all:
	$(MAKE) -C $(KSRC) M=$(KMOD_SRC) $(KMOD_OPTIONS) $(MAKECMDGOALS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)"

.PHONY: all clean load unload reload test

clean:
	$(MAKE) -C $(KSRC) M=$(KMOD_SRC) clean $(KMOD_OPTIONS)

load:
	modprobe mac80211
	insmod $(KMOD_SRC)/rtlwifi.ko
	insmod $(KMOD_SRC)/rtl_usb.ko
	insmod $(KMOD_SRC)/rtl8188eu/rtl8188eu.ko

unload:
	rmmod rtl8188eu || echo "rtl8188eu not loaded"
	rmmod rtl_usb   || echo "rtl_usb not loaded"
	rmmod rtlwifi   || echo "rtlwifi not loaded"

reload: unload load

test: all reload
