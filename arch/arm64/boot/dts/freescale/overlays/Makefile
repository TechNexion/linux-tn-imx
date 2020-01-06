# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_FSL_IMX8MM) += \
	imx8mm-pico-pi-mipi_5.dtbo \
	imx8mm-pico-pi-voicehat.dtbo \
	imx8mm-flex-pi-m4.dtbo \
	imx8mm-flex-pi-mipi5.dtbo \
	imx8mm-flex-pi-voicehat.dtbo \

targets += dtbs dtbs_install
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtbo
