# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_FSL_IMX8MM) += \
	imx8mm-pico-pi-mipi_5.dtbo \
	imx8mm-pico-pi-voicehat.dtbo \
	imx8mm-flex-pi-m4.dtbo \
	imx8mm-flex-pi-mipi5.dtbo \
	imx8mm-flex-pi-voicehat.dtbo \
	imx8mm-flex-wizard-m4.dtbo \
	imx8mm-flex-wizard-mipi5.dtbo \
	imx8mm-flex-wizard-mipi8.dtbo \
	imx8mm-flex-wizard-mipi2hdmi.dtbo \
	imx8mm-flex-wizard-lvds10.dtbo \
	imx8mm-flex-wizard-voicehat.dtbo \
	imx8mm-flex-wizard-ntag.dtbo \
	imx8mm-flex-wizard-clix1wifi.dtbo \
	imx8mm-flex-wizard-clix1nfc.dtbo \
	imx8mm-flex-wizard-clix2nfc.dtbo \

targets += dtbs dtbs_install
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtbo
