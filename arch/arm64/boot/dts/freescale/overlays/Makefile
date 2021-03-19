# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_MXC) += imx8mm-edm-g-wb-sn65dsi84-vl10112880.dtbo imx8mm-edm-g-wb-sn65dsi84-vl15613676.dtbo imx8mm-edm-g-wb-sn65dsi84-vl215192108.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mm-pico-pi-ili9881c.dtbo imx8mm-pico-pi-voicehat.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mm-pico-wizard-ili9881c.dtbo imx8mm-pico-wizard-voicehat.dtbo imx8mm-pico-wizard-mipi2hdmi-adv7535.dtbo \
			   imx8mm-pico-wizard-g080uan01.dtbo imx8mm-pico-wizard-g101uan02.dtbo \
			   imx8mm-pico-wizard-sn65dsi84-vl10112880.dtbo imx8mm-pico-wizard-sn65dsi84-vl15613676.dtbo imx8mm-pico-wizard-sn65dsi84-vl215192108.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wb-lvds-vl10112880.dtbo imx8mp-edm-g-wb-lvds-vl15613676.dtbo imx8mp-edm-g-wb-lvds-vl215192108.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-edm-wizard-mipi2hdmi-adv7535.dtbo imx8mq-edm-wizard-mipi-dcss-ili9881c.dtbo \
			    imx8mq-edm-wizard-mipi-dcss-g080uan01.dtbo imx8mq-edm-wizard-mipi-dcss-g101uan02.dtbo \
			    imx8mq-edm-wizard-sn65dsi84-vl10112880.dtbo imx8mq-edm-wizard-sn65dsi84-vl15613676.dtbo imx8mq-edm-wizard-sn65dsi84-vl215192108.dtbo \
			    imx8mq-edm-wizard-clix2nfc.dtbo imx8mq-edm-wizard-voicehat.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-pico-pi-mipi-dcss-ili9881c.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y        := $(dtbo-y)
clean-files     := *.dtbo
