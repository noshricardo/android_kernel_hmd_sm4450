#
# Makefile for kernel Silead Fingerprint drivers.
#
ifdef CONFIG_ARCH_MSM
	 ccflags-y += -DBSP_SIL_PLAT_QCOM
endif

ifdef CONFIG_ARCH_QCOM
	 ccflags-y += -DBSP_SIL_PLAT_QCOM
endif

ifdef CONFIG_ARCH_SPRD
	 ccflags-y += -DBSP_SIL_PLAT_SPRD
endif

ifdef CONFIG_ARCH_MTK_PROJECT
   #include $(srctree)/drivers/misc/mediatek/Makefile.custom
   ccflags-y += -DBSP_SIL_PLAT_MTK
   ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
   subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat
 ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
   subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
 endif
 ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
   include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
 endif
endif

ccflags-y += -DCONFIG_SILEAD_FP_PLATFORM
obj-m     += silead_fp.o

#silead-y := silead_fp.o
#obj-$(CONFIG_SILEAD_FP) += silead.o
#ifeq ($(CONFIG_SILEAD_FP),m)
#$(info "--------sileadfp IS BUILT AS DLKM")
#BOARD_VENDOR_KERNEL_MODULES +=  $(KERNEL_MODULES_OUT)/silead.ko
#endif

