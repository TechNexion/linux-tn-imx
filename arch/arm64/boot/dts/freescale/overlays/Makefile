# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wb-lvds-vl10112880.dtbo imx8mp-edm-g-wb-lvds-vl215192108.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-edm-wizard-mipi2hdmi-adv7535.dtbo imx8mq-edm-wizard-mipi-dcss-ili9881c.dtbo \
			    imx8mq-edm-wizard-mipi-dcss-g080uan01.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y        := $(dtbo-y)
clean-files     := *.dtbo
