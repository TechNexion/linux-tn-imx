# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_MXC) += imx8mm-edm-g-wb-sn65dsi84-vl10112880.dtbo imx8mm-edm-g-wb-sn65dsi84-vl15613676.dtbo imx8mm-edm-g-wb-sn65dsi84-vl215192108.dtbo \
			   imx8mm-edm-g-wb-tevi-ov5640.dtbo imx8mm-edm-g-wb-hdmi2mipi-tc358743.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mm-edm-g-wizard-sn65dsi84-vl10112880.dtbo imx8mm-edm-g-wizard-sn65dsi84-vl15613676.dtbo imx8mm-edm-g-wizard-sn65dsi84-vl215192108.dtbo \
			   imx8mm-edm-g-wizard-tevi-ov5640.dtbo imx8mm-edm-g-wizard-hdmi2mipi-tc358743.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mm-pico-pi-ili9881c.dtbo imx8mm-pico-pi-voicehat.dtbo \
			   imx8mm-pico-pi-clixnfc.dtbo imx8mm-pico-pi-tevi-ov5640.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mm-pico-wizard-ili9881c.dtbo imx8mm-pico-wizard-voicehat.dtbo imx8mm-pico-wizard-mipi2hdmi-adv7535.dtbo \
			   imx8mm-pico-wizard-g080uan01.dtbo imx8mm-pico-wizard-g101uan02.dtbo \
			   imx8mm-pico-wizard-clix1nfc.dtbo imx8mm-pico-wizard-clix2nfc.dtbo \
			   imx8mm-pico-wizard-sn65dsi84-vl10112880.dtbo imx8mm-pico-wizard-sn65dsi84-vl15613676.dtbo imx8mm-pico-wizard-sn65dsi84-vl215192108.dtbo \
			   imx8mm-pico-wizard-tevi-ov5640.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wb-lvds-vl10112880.dtbo imx8mp-edm-g-wb-lvds-vl15613676.dtbo \
				imx8mp-edm-g-wb-lvds-vl215192108.dtbo imx8mp-edm-g-wb-tevi-ov5640.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wb-hdmi2mipi-tc358743-dual.dtbo \
				imx8mp-edm-g-wb-hdmi2mipi-tc358743-single-cam1.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wizard-mipi2hdmi-adv7535.dtbo imx8mp-edm-g-wizard-lvds-vl10112880.dtbo imx8mp-edm-g-wizard-lvds-vl15613676.dtbo \
                                imx8mp-edm-g-wizard-lvds-vl215192108.dtbo imx8mp-edm-g-wizard-tevi-ov5640.dtbo imx8mp-edm-g-wizard-tevi-ar0521.dtbo \
                                imx8mp-edm-g-wizard-tevi-ar0144.dtbo imx8mp-edm-g-wizard-tevi-ar0234.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wizard-hdmi2mipi-tc358743-dual.dtbo imx8mp-edm-g-wizard-hdmi2mipi-tc358743-single-cam1.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mp-tek3-hdmi2mipi-tc358743-dual.dtbo imx8mp-tek3-hdmi2mipi-tc358743_1.dtbo imx8mp-tek3-hdmi2mipi-tc358743_2.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-edm-wizard-mipi2hdmi-adv7535.dtbo imx8mq-edm-wizard-ili9881c.dtbo \
			    imx8mq-edm-wizard-g080uan01.dtbo imx8mq-edm-wizard-g101uan02.dtbo \
			    imx8mq-edm-wizard-sn65dsi84-vl10112880.dtbo imx8mq-edm-wizard-sn65dsi84-vl15613676.dtbo imx8mq-edm-wizard-sn65dsi84-vl215192108.dtbo \
			    imx8mq-edm-wizard-clix2nfc.dtbo imx8mq-edm-wizard-voicehat.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-pico-pi-ili9881c.dtbo imx8mq-pico-pi-voicehat.dtbo \
			   imx8mq-pico-pi-clixnfc.dtbo imx8mq-pico-pi-tevi-ov5640.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mq-pico-wizard-ili9881c.dtbo imx8mq-pico-wizard-g080uan01.dtbo imx8mq-pico-wizard-g101uan02.dtbo \
			   imx8mq-pico-wizard-clix1nfc.dtbo imx8mq-pico-wizard-clix2nfc.dtbo \
			   imx8mq-pico-wizard-sn65dsi84-vl10112880.dtbo imx8mq-pico-wizard-sn65dsi84-vl15613676.dtbo imx8mq-pico-wizard-sn65dsi84-vl215192108.dtbo \
			   imx8mq-pico-wizard-mipi2hdmi-adv7535.dtbo imx8mq-pico-wizard-voicehat.dtbo imx8mq-pico-wizard-tevi-ov5640.dtbo
dtbo-$(CONFIG_ARCH_MXC) += imx8mn-edm-g-wb-sn65dsi84-vl10112880.dtbo imx8mn-edm-g-wb-sn65dsi84-vl15613676.dtbo imx8mn-edm-g-wb-sn65dsi84-vl215192108.dtbo \
			   imx8mn-edm-g-wb-tevi-ov5640.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y        := $(dtbo-y)
clean-files     := *.dtbo
