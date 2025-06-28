
FTS_FIRMWARE_PATH:=kernel_platform/msm-kernel/drivers/input/touchscreen/focaltech_ft8057s/firmware
PRODUCT_COPY_FILES += $(FTS_FIRMWARE_PATH)/focaltech_firmware.bin:vendor/firmware/focaltech_firmware.bin
FTS_INI_PATH:=kernel_platform/msm-kernel/drivers/input/touchscreen/focaltech_ft8057s/include/firmware/
PRODUCT_COPY_FILES += $(FTS_INI_PATH)/focaltech_ft8057s.ini:vendor/firmware/focaltech_ft8057s.ini


