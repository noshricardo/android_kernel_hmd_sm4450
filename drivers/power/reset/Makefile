# SPDX-License-Identifier: GPL-2.0
$(warning "Compile HMD_RAMDUMP_DISABLE $(HMD_RAMDUMP_DISABLE)")

ifeq ($(HMD_RAMDUMP_DISABLE), true)
subdir-ccflags-y += -DHMD_RAMDUMP_DISABLE
endif

obj-$(CONFIG_POWER_RESET_AS3722) += as3722-poweroff.o
obj-$(CONFIG_POWER_RESET_AT91_POWEROFF) += at91-poweroff.o
obj-$(CONFIG_POWER_RESET_AT91_RESET) += at91-reset.o
obj-$(CONFIG_POWER_RESET_AT91_SAMA5D2_SHDWC) += at91-sama5d2_shdwc.o
obj-$(CONFIG_POWER_RESET_AXXIA) += axxia-reset.o
obj-$(CONFIG_POWER_RESET_BRCMKONA) += brcm-kona-reset.o
obj-$(CONFIG_POWER_RESET_BRCMSTB) += brcmstb-reboot.o
obj-$(CONFIG_POWER_RESET_GEMINI_POWEROFF) += gemini-poweroff.o
obj-$(CONFIG_POWER_RESET_GPIO) += gpio-poweroff.o
obj-$(CONFIG_POWER_RESET_GPIO_RESTART) += gpio-restart.o
obj-$(CONFIG_POWER_RESET_HISI) += hisi-reboot.o
obj-${CONFIG_POWER_RESET_LINKSTATION} += linkstation-poweroff.o
obj-$(CONFIG_POWER_RESET_MSM) += msm-poweroff.o
obj-$(CONFIG_POWER_RESET_MT6323) += mt6323-poweroff.o
obj-$(CONFIG_POWER_RESET_OXNAS) += oxnas-restart.o
obj-$(CONFIG_POWER_RESET_QCOM_DOWNLOAD_MODE) += qcom-dload-mode.o
obj-$(CONFIG_POWER_RESET_QCOM_PON) += qcom-pon.o
obj-$(CONFIG_POWER_RESET_QCOM_REBOOT_REASON) += qcom-reboot-reason.o
obj-$(CONFIG_POWER_RESET_OCELOT_RESET) += ocelot-reset.o
obj-$(CONFIG_POWER_RESET_PIIX4_POWEROFF) += piix4-poweroff.o
obj-$(CONFIG_POWER_RESET_LTC2952) += ltc2952-poweroff.o
obj-$(CONFIG_POWER_RESET_QNAP) += qnap-poweroff.o
obj-$(CONFIG_POWER_RESET_RESTART) += restart-poweroff.o
obj-$(CONFIG_POWER_RESET_ST) += st-poweroff.o
obj-$(CONFIG_POWER_RESET_VERSATILE) += arm-versatile-reboot.o
obj-$(CONFIG_POWER_RESET_VEXPRESS) += vexpress-poweroff.o
obj-$(CONFIG_POWER_RESET_XGENE) += xgene-reboot.o
obj-$(CONFIG_POWER_RESET_KEYSTONE) += keystone-reset.o
obj-$(CONFIG_POWER_RESET_SYSCON) += syscon-reboot.o
obj-$(CONFIG_POWER_RESET_SYSCON_POWEROFF) += syscon-poweroff.o
obj-$(CONFIG_POWER_RESET_RMOBILE) += rmobile-reset.o
obj-$(CONFIG_POWER_RESET_ZX) += zx-reboot.o
obj-$(CONFIG_REBOOT_MODE) += reboot-mode.o
obj-$(CONFIG_SYSCON_REBOOT_MODE) += syscon-reboot-mode.o
obj-$(CONFIG_POWER_RESET_SC27XX) += sc27xx-poweroff.o
obj-$(CONFIG_NVMEM_REBOOT_MODE) += nvmem-reboot-mode.o
