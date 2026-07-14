#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>

#include "tevs_tbls.h"

/* Define host command register of TEVS information page */
#define HOST_COMMAND_TEVS_INFO_VERSION 							CCI_REG32(0x3000)
#define HOST_COMMAND_TEVS_INFO_VERSION_MSB 						CCI_REG16(0x3000)
#define HOST_COMMAND_TEVS_INFO_VERSION_LSB 						CCI_REG16(0x3002)
#define HOST_COMMAND_TEVS_BOOT_STATE 							CCI_REG16(0x3004)
#define HOST_COMMAND_TEVS_SENSOR_CHIP_ID                        CCI_REG16(0x3008)
#define HOST_COMMAND_TEVS_MODEL_NUMBER_0                        CCI_REG16(0x3020)
#define HOST_COMMAND_TEVS_MODEL_NUMBER_1                        CCI_REG16(0x3022)
#define HOST_COMMAND_TEVS_MODEL_NUMBER_2                        CCI_REG16(0x3024)

/* Define host command register of ISP control page */
#define HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH 					CCI_REG16(0x3100)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT 					CCI_REG16(0x3102)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT 					CCI_REG16(0x3104)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_SENSOR_MODE 				CCI_REG16(0x3106)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_THROUGHPUT 				CCI_REG16(0x3108)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS 					CCI_REG16(0x310A)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER	 		CCI_REG32(0x310C)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_MSB 		CCI_REG16(0x310C)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_LSB 		CCI_REG16(0x310E)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX	 			CCI_REG32(0x3110)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_MSB 			CCI_REG16(0x3110)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_LSB 			CCI_REG16(0x3112)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL 				CCI_REG16(0x3114)
#define HOST_COMMAND_ISP_CTRL_AE_MODE 							CCI_REG16(0x3116)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME	 						CCI_REG32(0x3118)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MSB 						CCI_REG16(0x3118)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_LSB 						CCI_REG16(0x311A)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX	 					CCI_REG32(0x311C)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_MSB 					CCI_REG16(0x311C)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_LSB 					CCI_REG16(0x311E)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN	 					CCI_REG32(0x3120)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_MSB 					CCI_REG16(0x3120)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_LSB 					CCI_REG16(0x3122)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN						 	CCI_REG16(0x3124)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MAX 						CCI_REG16(0x3126)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MIN 						CCI_REG16(0x3128)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME	 				CCI_REG32(0x312A)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_MSB 				CCI_REG16(0x312A)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_LSB 				CCI_REG16(0x312C)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_GAIN 					CCI_REG16(0x312E)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION 			CCI_REG16(0x3130)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MAX 		CCI_REG16(0x3132)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MIN 		CCI_REG16(0x3134)
#define HOST_COMMAND_ISP_CTRL_AWB_MODE 							CCI_REG16(0x3136)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP 							CCI_REG16(0x3138)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MAX 						CCI_REG16(0x313A)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MIN 						CCI_REG16(0x313C)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS 						CCI_REG16(0x313E)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MAX 					CCI_REG16(0x3140)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MIN 					CCI_REG16(0x3142)
#define HOST_COMMAND_ISP_CTRL_CONTRAST 							CCI_REG16(0x3144)
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MAX 						CCI_REG16(0x3146)
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MIN 						CCI_REG16(0x3148)
#define HOST_COMMAND_ISP_CTRL_SATURATION 						CCI_REG16(0x314A)
#define HOST_COMMAND_ISP_CTRL_SATURATION_MAX 					CCI_REG16(0x314C)
#define HOST_COMMAND_ISP_CTRL_SATURATION_MIN 					CCI_REG16(0x314E)
#define HOST_COMMAND_ISP_CTRL_GAMMA 							CCI_REG16(0x3150)
#define HOST_COMMAND_ISP_CTRL_GAMMA_MAX 						CCI_REG16(0x3152)
#define HOST_COMMAND_ISP_CTRL_GAMMA_MIN 						CCI_REG16(0x3154)
#define HOST_COMMAND_ISP_CTRL_DENOISE 							CCI_REG16(0x3156)
#define HOST_COMMAND_ISP_CTRL_DENOISE_MAX 						CCI_REG16(0x3158)
#define HOST_COMMAND_ISP_CTRL_DENOISE_MIN 						CCI_REG16(0x315A)
#define HOST_COMMAND_ISP_CTRL_SHARPEN 							CCI_REG16(0x315C)
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MAX 						CCI_REG16(0x315E)
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MIN 						CCI_REG16(0x3160)
#define HOST_COMMAND_ISP_CTRL_FLIP 								CCI_REG16(0x3162)
#define HOST_COMMAND_ISP_CTRL_EFFECT 							CCI_REG16(0x3164)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TYPE 						CCI_REG16(0x3166)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES 						CCI_REG16(0x3168)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MAX 					CCI_REG16(0x316A)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MIN 					CCI_REG16(0x316C)
#define HOST_COMMAND_ISP_CTRL_CT_X 								CCI_REG16(0x316E)
#define HOST_COMMAND_ISP_CTRL_CT_Y 								CCI_REG16(0x3170)
#define HOST_COMMAND_ISP_CTRL_CT_MAX 							CCI_REG16(0x3172)
#define HOST_COMMAND_ISP_CTRL_CT_MIN 							CCI_REG16(0x3174)
#define HOST_COMMAND_ISP_CTRL_SYSTEM_START 						CCI_REG16(0x3176)
#define HOST_COMMAND_ISP_CTRL_ISP_RESET 						CCI_REG16(0x3178)
#define HOST_COMMAND_ISP_CTRL_TRIGGER_MODE 						CCI_REG16(0x317A)
#define HOST_COMMAND_ISP_CTRL_FLICK_CTRL					 	CCI_REG16(0x317C)
#define HOST_COMMAND_ISP_CTRL_MIPI_FREQ 						CCI_REG16(0x317E)
#define HOST_COMMAND_ISP_CTRL_JPEG_QUAL							CCI_REG16(0x3180)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_MIPI_CTRL 				CCI_REG16(0x3182)

/* Define host command register of ISP bootdata page */
#define HOST_COMMAND_ISP_BOOTDATA_1								CCI_REG16(0x4000)
#define HOST_COMMAND_ISP_BOOTDATA_2								CCI_REG16(0x4002)
#define HOST_COMMAND_ISP_BOOTDATA_3								CCI_REG16(0x4004)
#define HOST_COMMAND_ISP_BOOTDATA_4								CCI_REG16(0x4006)
#define HOST_COMMAND_ISP_BOOTDATA_5								CCI_REG16(0x4008)
#define HOST_COMMAND_ISP_BOOTDATA_6								CCI_REG16(0x400A)
#define HOST_COMMAND_ISP_BOOTDATA_7								CCI_REG16(0x400C)
#define HOST_COMMAND_ISP_BOOTDATA_8								CCI_REG16(0x400E)
#define HOST_COMMAND_ISP_BOOTDATA_9								CCI_REG16(0x4010)
#define HOST_COMMAND_ISP_BOOTDATA_10							CCI_REG16(0x4012)
#define HOST_COMMAND_ISP_BOOTDATA_11							CCI_REG16(0x4014)
#define HOST_COMMAND_ISP_BOOTDATA_12							CCI_REG16(0x4016)
#define HOST_COMMAND_ISP_BOOTDATA_13							CCI_REG16(0x4018)
#define HOST_COMMAND_ISP_BOOTDATA_14							CCI_REG16(0x401A)
#define HOST_COMMAND_ISP_BOOTDATA_15							CCI_REG16(0x401C)
#define HOST_COMMAND_ISP_BOOTDATA_16							CCI_REG16(0x401E)
#define HOST_COMMAND_ISP_BOOTDATA_17							CCI_REG16(0x4020)
#define HOST_COMMAND_ISP_BOOTDATA_18							CCI_REG16(0x4022)
#define HOST_COMMAND_ISP_BOOTDATA_19							CCI_REG16(0x4024)
#define HOST_COMMAND_ISP_BOOTDATA_20							CCI_REG16(0x4026)
#define HOST_COMMAND_ISP_BOOTDATA_21							CCI_REG16(0x4028)
#define HOST_COMMAND_ISP_BOOTDATA_22							CCI_REG16(0x402A)
#define HOST_COMMAND_ISP_BOOTDATA_23							CCI_REG16(0x402C)
#define HOST_COMMAND_ISP_BOOTDATA_24							CCI_REG16(0x402E)
#define HOST_COMMAND_ISP_BOOTDATA_25							CCI_REG16(0x4030)
#define HOST_COMMAND_ISP_BOOTDATA_26							CCI_REG16(0x4032)
#define HOST_COMMAND_ISP_BOOTDATA_27							CCI_REG16(0x4034)
#define HOST_COMMAND_ISP_BOOTDATA_28							CCI_REG16(0x4036)
#define HOST_COMMAND_ISP_BOOTDATA_29							CCI_REG16(0x4038)
#define HOST_COMMAND_ISP_BOOTDATA_30							CCI_REG16(0x403A)
#define HOST_COMMAND_ISP_BOOTDATA_31							CCI_REG16(0x403C)
#define HOST_COMMAND_ISP_BOOTDATA_32							CCI_REG16(0x403E)
#define HOST_COMMAND_ISP_BOOTDATA_33							CCI_REG16(0x4040)
#define HOST_COMMAND_ISP_BOOTDATA_34							CCI_REG16(0x4042)
#define HOST_COMMAND_ISP_BOOTDATA_35							CCI_REG16(0x4044)
#define HOST_COMMAND_ISP_BOOTDATA_36							CCI_REG16(0x4046)
#define HOST_COMMAND_ISP_BOOTDATA_37							CCI_REG16(0x4048)
#define HOST_COMMAND_ISP_BOOTDATA_38							CCI_REG16(0x404A)
#define HOST_COMMAND_ISP_BOOTDATA_39							CCI_REG16(0x404C)
#define HOST_COMMAND_ISP_BOOTDATA_40							CCI_REG16(0x404E)
#define HOST_COMMAND_ISP_BOOTDATA_41							CCI_REG16(0x4050)
#define HOST_COMMAND_ISP_BOOTDATA_42							CCI_REG16(0x4052)
#define HOST_COMMAND_ISP_BOOTDATA_43							CCI_REG16(0x4054)
#define HOST_COMMAND_ISP_BOOTDATA_44							CCI_REG16(0x4056)
#define HOST_COMMAND_ISP_BOOTDATA_45							CCI_REG16(0x4058)
#define HOST_COMMAND_ISP_BOOTDATA_46							CCI_REG16(0x405A)
#define HOST_COMMAND_ISP_BOOTDATA_47							CCI_REG16(0x405C)
#define HOST_COMMAND_ISP_BOOTDATA_48							CCI_REG16(0x405E)
#define HOST_COMMAND_ISP_BOOTDATA_49							CCI_REG16(0x4060)
#define HOST_COMMAND_ISP_BOOTDATA_50							CCI_REG16(0x4062)
#define HOST_COMMAND_ISP_BOOTDATA_51							CCI_REG16(0x4064)
#define HOST_COMMAND_ISP_BOOTDATA_52							CCI_REG16(0x4066)
#define HOST_COMMAND_ISP_BOOTDATA_53							CCI_REG16(0x4068)
#define HOST_COMMAND_ISP_BOOTDATA_54							CCI_REG16(0x406A)
#define HOST_COMMAND_ISP_BOOTDATA_55							CCI_REG16(0x406C)
#define HOST_COMMAND_ISP_BOOTDATA_56							CCI_REG16(0x406E)
#define HOST_COMMAND_ISP_BOOTDATA_57							CCI_REG16(0x4070)
#define HOST_COMMAND_ISP_BOOTDATA_58							CCI_REG16(0x4072)
#define HOST_COMMAND_ISP_BOOTDATA_59							CCI_REG16(0x4074)
#define HOST_COMMAND_ISP_BOOTDATA_60							CCI_REG16(0x4076)
#define HOST_COMMAND_ISP_BOOTDATA_61							CCI_REG16(0x4078)
#define HOST_COMMAND_ISP_BOOTDATA_62							CCI_REG16(0x407A)
#define HOST_COMMAND_ISP_BOOTDATA_63							CCI_REG16(0x407C)

/* Define special method for controlling ISP with I2C */
#define HOST_COMMAND_ISP_CTRL_I2C_ADDR							CCI_REG16(0xF000)
#define HOST_COMMAND_ISP_CTRL_I2C_DATA							CCI_REG16(0xF002)

#define TEVS_BRIGHTNESS 						HOST_COMMAND_ISP_CTRL_BRIGHTNESS
#define TEVS_BRIGHTNESS_MAX 					HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MAX
#define TEVS_BRIGHTNESS_MIN 					HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MIN
#define TEVS_BRIGHTNESS_MASK 					(0xFFFF)
#define TEVS_CONTRAST 							HOST_COMMAND_ISP_CTRL_CONTRAST
#define TEVS_CONTRAST_MAX 						HOST_COMMAND_ISP_CTRL_CONTRAST_MAX
#define TEVS_CONTRAST_MIN 						HOST_COMMAND_ISP_CTRL_CONTRAST_MIN
#define TEVS_CONTRAST_MASK 						(0xFFFF)
#define TEVS_SATURATION 						HOST_COMMAND_ISP_CTRL_SATURATION
#define TEVS_SATURATION_MAX 					HOST_COMMAND_ISP_CTRL_SATURATION_MAX
#define TEVS_SATURATION_MIN 					HOST_COMMAND_ISP_CTRL_SATURATION_MIN
#define TEVS_SATURATION_MASK 					(0xFFFF)
#define TEVS_AWB_CTRL_MODE 						HOST_COMMAND_ISP_CTRL_AWB_MODE
#define TEVS_AWB_CTRL_MODE_MASK 				(0x00FF)
#define TEVS_AWB_CTRL_MODE_MANUAL_TEMP 			(7U << 0)
#define TEVS_AWB_CTRL_MODE_AUTO 				(15U << 0)
#define TEVS_AWB_CTRL_MODE_MANUAL_TEMP_IDX 		(0U << 0)
#define TEVS_AWB_CTRL_MODE_AUTO_IDX 			(1U << 0)
#define TEVS_GAMMA 								HOST_COMMAND_ISP_CTRL_GAMMA
#define TEVS_GAMMA_MAX 							HOST_COMMAND_ISP_CTRL_GAMMA_MAX
#define TEVS_GAMMA_MIN 							HOST_COMMAND_ISP_CTRL_GAMMA_MIN
#define TEVS_GAMMA_MASK 						(0xFFFF)
#define TEVS_AE_AUTO_EXP_TIME_UPPER				HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER
#define TEVS_AE_AUTO_EXP_TIME_MAX				HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX
#define TEVS_AE_AUTO_EXP_TIME_MASK				(0xFFFFFFFF)
#define TEVS_AE_MANUAL_EXP_TIME 				HOST_COMMAND_ISP_CTRL_EXP_TIME
#define TEVS_AE_MANUAL_EXP_TIME_MAX 			HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX
#define TEVS_AE_MANUAL_EXP_TIME_MIN 			HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN
#define TEVS_AE_MANUAL_EXP_TIME_MASK 			(0xFFFFFFFF)
#define TEVS_AE_MANUAL_GAIN 					HOST_COMMAND_ISP_CTRL_EXP_GAIN
#define TEVS_AE_MANUAL_GAIN_MAX 				HOST_COMMAND_ISP_CTRL_EXP_GAIN_MAX
#define TEVS_AE_MANUAL_GAIN_MIN 				HOST_COMMAND_ISP_CTRL_EXP_GAIN_MIN
#define TEVS_AE_MANUAL_GAIN_MASK 				(0x00FF)
#define TEVS_ORIENTATION 						HOST_COMMAND_ISP_CTRL_FLIP
#define TEVS_ORIENTATION_HFLIP_BIT 				(0U)
#define TEVS_ORIENTATION_HFLIP 					BIT(TEVS_ORIENTATION_HFLIP_BIT)
#define TEVS_ORIENTATION_VFLIP_BIT 				(1U)
#define TEVS_ORIENTATION_VFLIP 					BIT(TEVS_ORIENTATION_VFLIP_BIT)
#define TEVS_FLICK_CTRL    						HOST_COMMAND_ISP_CTRL_FLICK_CTRL
#define TEVS_FLICK_CTRL_MASK					(0xFFFF) // TEVS_REG_16BIT(0x5440)
#define TEVS_FLICK_CTRL_FREQ(n)					((n) << 8)
#define TEVS_FLICK_CTRL_ETC_IHDR_UP				BIT(6)
#define TEVS_FLICK_CTRL_ETC_DIS					BIT(5)
#define TEVS_FLICK_CTRL_FRC_OVERRIDE_MAX_ET		BIT(4)
#define TEVS_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define TEVS_FLICK_CTRL_FRC_EN					BIT(2)
#define TEVS_FLICK_CTRL_MODE_MASK				(3U << 0)
#define TEVS_FLICK_CTRL_MODE_DISABLED			(0U << 0)
#define TEVS_FLICK_CTRL_MODE_MANUAL				(1U << 0)
#define TEVS_FLICK_CTRL_MODE_AUTO				(2U << 0)
#define TEVS_FLICK_CTRL_FREQ_MASK			    (0xFF00)
#define TEVS_FLICK_CTRL_MODE_50HZ				(TEVS_FLICK_CTRL_FREQ(50) | TEVS_FLICK_CTRL_MODE_MANUAL)
#define TEVS_FLICK_CTRL_MODE_60HZ				(TEVS_FLICK_CTRL_FREQ(60) | TEVS_FLICK_CTRL_MODE_MANUAL)
#define TEVS_AWB_MANUAL_TEMP 					HOST_COMMAND_ISP_CTRL_AWB_TEMP
#define TEVS_AWB_MANUAL_TEMP_MAX 				HOST_COMMAND_ISP_CTRL_AWB_TEMP_MAX
#define TEVS_AWB_MANUAL_TEMP_MIN 				HOST_COMMAND_ISP_CTRL_AWB_TEMP_MIN
#define TEVS_AWB_MANUAL_TEMP_MASK 				(0xFFFF)
#define TEVS_SHARPEN 							HOST_COMMAND_ISP_CTRL_SHARPEN
#define TEVS_SHARPEN_MAX 						HOST_COMMAND_ISP_CTRL_SHARPEN_MAX
#define TEVS_SHARPEN_MIN 						HOST_COMMAND_ISP_CTRL_SHARPEN_MIN
#define TEVS_SHARPEN_MASK 						(0xFFFF)
#define TEVS_BACKLIGHT_COMPENSATION 			HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION
#define TEVS_BACKLIGHT_COMPENSATION_MAX 		HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MAX
#define TEVS_BACKLIGHT_COMPENSATION_MIN 		HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MIN
#define TEVS_BACKLIGHT_COMPENSATION_MASK 		(0xFFFF)
#define TEVS_DZ_TGT_FCT 						HOST_COMMAND_ISP_CTRL_ZOOM_TIMES
#define TEVS_DZ_TGT_FCT_MAX 					HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MAX
#define TEVS_DZ_TGT_FCT_MIN 					HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MIN
#define TEVS_DZ_TGT_FCT_MASK 					(0xFFFF)
#define TEVS_SFX_MODE 							HOST_COMMAND_ISP_CTRL_EFFECT
#define TEVS_SFX_MODE_SFX_MASK 					(0x00FF)
#define TEVS_SFX_MODE_SFX_NORMAL 				(0U << 0)
#define TEVS_SFX_MODE_SFX_BW 					(3U << 0)
#define TEVS_SFX_MODE_SFX_GRAYSCALE 			(6U << 0)
#define TEVS_SFX_MODE_SFX_NEGATIVE 				(7U << 0)
#define TEVS_SFX_MODE_SFX_SKETCH 				(15U << 0)
#define TEVS_SFX_MODE_SFX_NORMAL_IDX 			(0U << 0)
#define TEVS_SFX_MODE_SFX_BW_IDX 				(1U << 0)
#define TEVS_SFX_MODE_SFX_GRAYSCALE_IDX 		(2U << 0)
#define TEVS_SFX_MODE_SFX_NEGATIVE_IDX 			(3U << 0)
#define TEVS_SFX_MODE_SFX_SKETCH_IDX 			(4U << 0)
#define TEVS_AE_CTRL_MODE 						HOST_COMMAND_ISP_CTRL_AE_MODE
#define TEVS_AE_CTRL_MODE_MASK 					(0x00FF)
#define TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN 		(0U << 0)
#define TEVS_AE_CTRL_AUTO_GAIN 					(9U << 0)
#define TEVS_AE_CTRL_FULL_AUTO 					(12U << 0)
#define TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN_IDX 	(0U << 0)
#define TEVS_AE_CTRL_FULL_AUTO_IDX 				(1U << 0)
#define TEVS_AE_CTRL_AUTO_GAIN_IDX				(2U << 0)
#define TEVS_DZ_CT_X 							HOST_COMMAND_ISP_CTRL_CT_X
#define TEVS_DZ_CT_Y 							HOST_COMMAND_ISP_CTRL_CT_Y
#define TEVS_DZ_CT_MASK 						(0xFFFF)
#define TEVS_DZ_CT_MAX 							HOST_COMMAND_ISP_CTRL_CT_MAX
#define TEVS_DZ_CT_MIN 							HOST_COMMAND_ISP_CTRL_CT_MIN
#define TEVS_BSL_MODE_NORMAL_IDX				(0U << 0)
#define TEVS_BSL_MODE_FLASH_IDX 				(1U << 0)
#define TEVS_MAX_FPS							HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS
#define TEVS_MAX_FPS_MASK 						(0x00FF)
#define TEVS_DENOISE							HOST_COMMAND_ISP_CTRL_DENOISE
#define TEVS_DENOISE_MAX 						HOST_COMMAND_ISP_CTRL_DENOISE_MAX
#define TEVS_DENOISE_MIN 						HOST_COMMAND_ISP_CTRL_DENOISE_MIN
#define TEVS_DENOISE_MASK 						(0xFFFF)
#define TEVS_TRIGGER_MODE						HOST_COMMAND_ISP_CTRL_TRIGGER_MODE
#define TEVS_TRIGGER_MODE_MASK		 			(0x0003)
#define TEVS_TRIGGER_MODE_DISABLE				(0U << 0)
#define TEVS_TRIGGER_MODE_SYNC					(1U << 0)
#define TEVS_TRIGGER_MODE_PERIODIC				(2U << 0)
#define TEVS_TRIGGER_MODE_NON_PERIODIC			(3U << 0)
#define TEVS_TRIGGER_MODE_DISABLE_IDX			(0U << 0)
#define TEVS_TRIGGER_MODE_SYNC_IDX				(1U << 0)
#define TEVS_TRIGGER_MODE_PERIODIC_IDX			(2U << 0)
#define TEVS_TRIGGER_MODE_NON_PERIODIC_IDX		(3U << 0)

#define V4L2_CID_USER_TEVS_BASE				(V4L2_CID_USER_BASE + 0x2000)
#define V4L2_CID_TEVS_BSL_MODE				(V4L2_CID_USER_TEVS_BASE + 0)
#define V4L2_CID_TEVS_MAX_FPS				(V4L2_CID_USER_TEVS_BASE + 1)
#define V4L2_CID_TEVS_DENOISE				(V4L2_CID_USER_TEVS_BASE + 2)
#define V4L2_CID_TEVS_AE_EXP_TIME_UPPER		(V4L2_CID_USER_TEVS_BASE + 3)
#define V4L2_CID_TEVS_AE_EXP_TIME_MAX		(V4L2_CID_USER_TEVS_BASE + 4)
#define V4L2_CID_TEVS_TRIGGER_MODE			(V4L2_CID_USER_TEVS_BASE + 5)

#define DEFAULT_HEADER_VERSION 3
#define TEVS_BOOT_TIME						(250)
#define TOTAL_MICROSEC_PERSEC				(1000000)

#define TEVS_IMG_FORMAT_UYVY				(0x50)

#define TEVS_LINK_FREQUENCY_DEFAULT			400000000ull
#define TEVS_PIXEL_RATE_DEFAULT				200000000ull

#define TEVS_CONTINUOUS_CLOCK_DEFAULT 		(0)

struct header_info {
	u8 header_version;
	u16 content_offset;
	u16 sensor_type;
	u8 sensor_fuseid[16];
	u8 product_name[64];
	u8 lens_id[16];
	u16 fix_checksum;
	u8 tn_fw_version[2];
	u16 vendor_fw_version;
	u16 custom_number;
	u8 build_year;
	u8 build_month;
	u8 build_day;
	u8 build_hour;
	u8 build_minute;
	u8 build_second;
	u16 mipi_datarate;
	u32 content_len;
	u16 content_checksum;
	u16 total_checksum;
} __attribute__((packed));

struct tevs {
	struct v4l2_subdev v4l2_subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;

	struct regmap *regmap;
	struct header_info *header_info;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *host_pwdn_gpio;
	struct gpio_desc *standby_gpio;

	u16 chip_id;
	int data_lanes;
	int continuous_clock;
	int data_frequency;
	u8 selected_mode;
	u8 selected_sensor;
	bool supports_over_4k_res;
	bool hw_reset_mode;
	int trigger_mode;
	char *sensor_name;
	int vc_id;
	unsigned int fps;

	/* V4L2 Controls */
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *awb;
	struct v4l2_ctrl *gamma;
	struct v4l2_ctrl *exp_time;
	struct v4l2_ctrl *exp_gain;
	struct v4l2_ctrl *alg_gain;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *flick;
	struct v4l2_ctrl *wb_temp;
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *backlight_comp;
	struct v4l2_ctrl *colorfx;
	struct v4l2_ctrl *ae;
	struct v4l2_ctrl *pan;
	struct v4l2_ctrl *tilt;
	struct v4l2_ctrl *zoom;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *bsl;
	struct v4l2_ctrl *max_fps;
	struct v4l2_ctrl *denoise;
	struct v4l2_ctrl *ae_exp_upper;
	struct v4l2_ctrl *ae_exp_max;
	struct v4l2_ctrl *trigger;
};

static const struct regmap_config tevs_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int tevs_check_trigger_mode(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u64 val;
	int ret = 0;

	dev_dbg(&client->dev, "%s()\n", __func__);

	cci_read(tevs->regmap, TEVS_TRIGGER_MODE, &val, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "check trigger mode fail\n");
		return ret;
	}

	if ((val & TEVS_TRIGGER_MODE_MASK) == TEVS_TRIGGER_MODE_DISABLE)
		return 0;
	else
		return 1;
}

static int tevs_check_version(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u64 val;
	int ret = 0;

	ret = cci_read(tevs->regmap, HOST_COMMAND_TEVS_INFO_VERSION, &val,
		       NULL);
	if (ret < 0) {
		dev_err(&client->dev, "can't check version\n");
		return ret;
	}
	dev_info(&client->dev, "Version:%d.%d.%d.%d\n", (u8)(val >> 24) & 0xFF,
		 (u8)(val >> 16) & 0xFF, (u8)(val >> 8) & 0xFF,
		 (u8)(val & 0xFF));

	return 0;
}

static int tevs_load_header_info(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	struct header_info *header = tevs->header_info;
	u8 header_ver;
	u64 val;
	int ret = 0;

	ret = cci_read(tevs->regmap, HOST_COMMAND_ISP_BOOTDATA_1, &val, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "can't recognize header info\n");
		return ret;
	}

	header_ver = (val >> 8) & 0xFF;

	if (header_ver == DEFAULT_HEADER_VERSION) {
		ret = regmap_bulk_read(tevs->regmap,
				       HOST_COMMAND_ISP_BOOTDATA_1,
				       (u8 *)header,
				       sizeof(struct header_info));
		if (ret < 0) {
			dev_err(&client->dev, "read header info fail\n");
			return ret;
		}

		dev_info(&client->dev,
			 "Product:%s, HeaderVer:%d, MIPI_Rate:%d\n",
			 header->product_name, header->header_version,
			 header->mipi_datarate);

		dev_dbg(&client->dev,
			"content checksum: %x, content length: %d\n",
			header->content_checksum, header->content_len);

		return 0;
	} else {
		dev_err(&client->dev,
			"can't recognize header version number '0x%X'\n",
			header_ver);
		return -EINVAL;
	}
}

static int tevs_get_chip_id(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u64 val;
	int ret = 0;

	ret = cci_read(tevs->regmap, HOST_COMMAND_TEVS_SENSOR_CHIP_ID, &val,
		       NULL);
	if (ret < 0) {
		dev_err(&client->dev, "Can't get chip ID. ret = %d.\n", ret);
		return ret;
	}

	tevs->chip_id = val & 0xFFFF;
	dev_info(&client->dev, "Chip ID: 0x%.4X\n", tevs->chip_id);
	return 0;
}

static int tevs_standby(struct tevs *tevs, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u64 val = 0xFFFF;
	int timeout = 0;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	if (enable == 1) {
		cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_SYSTEM_START,
			  0x0000, NULL);
		usleep_range(9000, 10000);
		while (timeout < 100) {
			cci_read(tevs->regmap,
				 HOST_COMMAND_ISP_CTRL_SYSTEM_START, &val,
				 NULL);
			if ((val & 0xFF00) == 0x0000)
				break;
			if (++timeout >= 100) {
				dev_err(&client->dev, "timeout: line[%d]v=%x\n",
					__LINE__, (u16)val);
				return -EINVAL;
			}
			usleep_range(9000, 10000);
		}
		dev_dbg(&client->dev, "sensor standby\n");
	} else {
		cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_SYSTEM_START,
			  0x0001, NULL);
		usleep_range(9000, 10000);
		while (timeout < 100) {
			cci_read(tevs->regmap,
				 HOST_COMMAND_ISP_CTRL_SYSTEM_START, &val,
				 NULL);
			if ((val & 0xFF00) == 0x0100)
				break;
			if (++timeout >= 100) {
				dev_err(&client->dev, "timeout: line[%d]v=%x\n",
					__LINE__, (u16)val);
				return -EINVAL;
			}
			usleep_range(9000, 10000);
		}
		dev_dbg(&client->dev, "sensor wakeup\n");
	}

	return 0;
}

static int tevs_check_boot_state(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u16 boot_state;
	u8 timeout = 0;
	u64 val;
	int ret = 0;

	while (timeout < 20) {
		cci_read(tevs->regmap, HOST_COMMAND_TEVS_BOOT_STATE, &val,
			 NULL);
		boot_state = val & 0xFFFF;
		if (boot_state == 0x08)
			break;
		dev_dbg(&client->dev, "bootup state: 0x%04X\n", boot_state);
		if (++timeout >= 20) {
			dev_err(&client->dev, "bootup timeout: state: 0x%04X\n",
				boot_state);
			ret = -EBUSY;
		}
		msleep(50);
	}

	return ret;
}

static int tevs_set_stream(struct v4l2_subdev *sub_dev, int enable)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	u64 val;
	int ret = 0;

	if (tevs->selected_mode >=
	    tevs_sensor_table[tevs->selected_sensor].res_list_size)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s() enable [%x]\n", __func__, enable);

	if (enable == 0) {
		if (!(tevs->hw_reset_mode | tevs_check_trigger_mode(tevs)))
			ret = tevs_standby(tevs, 1);

		if (tevs->continuous_clock) {
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL,
				  0x10 | (TEVS_CONTINUOUS_CLOCK_DEFAULT  << 5) |
					  (tevs->data_lanes),
				  NULL);
		}
	} else {
		if (!(tevs->hw_reset_mode | tevs_check_trigger_mode(tevs)))
			ret = tevs_standby(tevs, 0);

		if (ret == 0) {
			dev_dbg(sub_dev->dev, "%s() width=%d, height=%d\n",
				__func__,
				tevs_sensor_table[tevs->selected_sensor]
					.res_list[tevs->selected_mode]
					.width,
				tevs_sensor_table[tevs->selected_sensor]
					.res_list[tevs->selected_mode]
					.height);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT,
				  TEVS_IMG_FORMAT_UYVY, NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL,
				  0x10 | (tevs->continuous_clock << 5) |
					  (tevs->data_lanes),
				  NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_MIPI_CTRL,
				  tevs->vc_id, NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_SENSOR_MODE,
				  tevs_sensor_table[tevs->selected_sensor]
					  .res_list[tevs->selected_mode]
					  .mode,
				  NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH,
				  tevs_sensor_table[tevs->selected_sensor]
					  .res_list[tevs->selected_mode]
					  .width,
				  NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT,
				  tevs_sensor_table[tevs->selected_sensor]
					  .res_list[tevs->selected_mode]
					  .height,
				  NULL);
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS,
				  tevs->fps, NULL);
			if (tevs->max_fps)
				tevs->max_fps->cur.val = tevs->fps;
			cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME, &val,
				 NULL);
			tevs->exp_time->cur.val = val &
						  TEVS_AE_MANUAL_EXP_TIME_MASK;
			cci_read(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_UPPER,
				 &val, NULL);
			tevs->ae_exp_upper->cur.val =
				val & TEVS_AE_MANUAL_EXP_TIME_MASK;
			cci_read(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_MAX, &val,
				 NULL);
			tevs->ae_exp_max->cur.val =
				val & TEVS_AE_MANUAL_EXP_TIME_MASK;
		}
	}

	return ret;
}

static int tevs_enum_mbus_code(struct v4l2_subdev *sub_dev,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	if (code->pad ||
	    code->index >=
		    tevs_sensor_table[tevs->selected_sensor].code_list_size)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s(), index [%u]\n", __func__, code->index);

	code->code =
		tevs_sensor_table[tevs->selected_sensor].code_list[code->index];

	dev_dbg(sub_dev->dev, "%s(), code [0x%x]\n", __func__, code->code);

	return 0;
}

static int tevs_enum_frame_size(struct v4l2_subdev *sub_dev,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);

	if ((fse->pad != 0) ||
	    (fse->index >=
	     tevs_sensor_table[tevs->selected_sensor].res_list_size))
		return -EINVAL;

	if (fse->code != tevs_sensor_table[tevs->selected_sensor].code_list[0])
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s(), index [%u]\n", __func__, fse->index);

	if (!tevs->supports_over_4k_res &&
	    tevs_sensor_table[tevs->selected_sensor].res_list[fse->index].width >
		    4096)
		return -EINVAL;

	fse->min_width = fse->max_width =
		tevs_sensor_table[tevs->selected_sensor]
			.res_list[fse->index]
			.width;
	fse->min_height = fse->max_height =
		tevs_sensor_table[tevs->selected_sensor]
			.res_list[fse->index]
			.height;

	dev_dbg(sub_dev->dev, "%s(), w [%u] h [%u]\n", __func__, fse->min_width,
		fse->min_height);

	return 0;
}

static int tevs_enum_frame_interval(struct v4l2_subdev *sub_dev,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	struct sensor_info *sensor = &tevs_sensor_table[tevs->selected_sensor];
	int i;

	if (fie->pad != 0)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s() index [%u]\n", __func__, fie->index);

	for (i = 0; i < sensor->res_list_size; i++) {
		if (fie->width == sensor->res_list[i].width &&
		    fie->height == sensor->res_list[i].height) {
			if (fie->index >= sensor->res_list[i].framerates_size)
				return -EINVAL;

			fie->interval.numerator = 1;
			fie->interval.denominator =
				sensor->res_list[i].framerates[fie->index];

			dev_dbg(sub_dev->dev, "%s() frame rate [%u]\n", __func__,
				fie->interval.denominator);

			return 0;
		}
	}

	return -EINVAL;
}

static int tevs_get_fmt(struct v4l2_subdev *sub_dev,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);

	if (format->pad != 0)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s() which [%d]\n", __func__, format->which);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_state_get_format(sd_state, format->pad);
	else
		fmt = &tevs->fmt;

	dev_dbg(sub_dev->dev,
		"%s() w [%u] h [%u] code [0x%x] colorspace [%u]\n", __func__,
		fmt->width, fmt->height, fmt->code, fmt->colorspace);

	memmove(mbus_fmt, fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tevs_set_fmt(struct v4l2_subdev *sub_dev,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	int i;

	if (format->pad != 0)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s(): which [%d]\n", __func__, format->which);

	for (i = 0; i < tevs_sensor_table[tevs->selected_sensor].res_list_size;
	     i++) {
		if (mbus_fmt->width == tevs_sensor_table[tevs->selected_sensor]
					       .res_list[i]
					       .width &&
		    mbus_fmt->height == tevs_sensor_table[tevs->selected_sensor]
						.res_list[i]
						.height)
			break;
	}

	if (i >= tevs_sensor_table[tevs->selected_sensor].res_list_size) {
		return -EINVAL;
	}

	tevs->selected_mode = i;
	dev_dbg(sub_dev->dev, "%s() selected mode index [%d]\n", __func__,
		tevs->selected_mode);

	mbus_fmt->width =
		tevs_sensor_table[tevs->selected_sensor].res_list[i].width;
	mbus_fmt->height =
		tevs_sensor_table[tevs->selected_sensor].res_list[i].height;
	mbus_fmt->code = tevs_sensor_table[tevs->selected_sensor].code_list[0];
	mbus_fmt->field = V4L2_FIELD_NONE;
	mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
	mbus_fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(mbus_fmt->colorspace);
	mbus_fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mbus_fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(mbus_fmt->colorspace);
	memset(mbus_fmt->reserved, 0, sizeof(mbus_fmt->reserved));

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_state_get_format(sd_state, format->pad);
	else
		fmt = &tevs->fmt;

	memmove(fmt, mbus_fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tevs_get_selection(struct v4l2_subdev *sub_dev,
			      struct v4l2_subdev_state *sub_state,
			      struct v4l2_subdev_selection *sel)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	int index = tevs_sensor_table[tevs->selected_sensor].res_list_size - 1;
	struct v4l2_mbus_framefmt *fmt;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
			fmt = v4l2_subdev_state_get_format(sub_state, sel->pad);
		else
			fmt = &tevs->fmt;

		if (!fmt)
			return -EINVAL;

		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = fmt->width;
		sel->r.height = fmt->height;

		dev_dbg(sub_dev->dev, "%s() crop selection [%d, %d, %d, %d]\n",
			__func__, sel->r.top, sel->r.left, sel->r.width,
			sel->r.height);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = tevs_sensor_table[tevs->selected_sensor].res_list[index].width;
		sel->r.height = tevs_sensor_table[tevs->selected_sensor].res_list[index].height;

		dev_dbg(sub_dev->dev, "%s() bounds selection [%d, %d, %d, %d]\n",
			__func__, sel->r.top, sel->r.left, sel->r.width,
			sel->r.height);
		return 0;
	}

	return -EINVAL;
}

static int tevs_get_frame_interval(struct v4l2_subdev *sub_dev,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);

	if (fi->pad != 0)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	fi->interval.numerator = 1;
	fi->interval.denominator = tevs->fps;
	dev_dbg(sub_dev->dev, "fps = %d\n", tevs->fps);

	return 0;
}

static int tevs_set_frame_interval(struct v4l2_subdev *sub_dev,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	unsigned int max_fps, min_fps;
	unsigned int fps =
		fi->interval.numerator ?
			fi->interval.denominator / fi->interval.numerator :
			fi->interval.denominator;

	if (fi->pad != 0)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	max_fps = tevs_sensor_table[tevs->selected_sensor]
			  .res_list[tevs->selected_mode]
			  .framerates[0];
	min_fps = tevs_sensor_table[tevs->selected_sensor]
			  .res_list[tevs->selected_mode]
			  .framerates[tevs_sensor_table[tevs->selected_sensor]
					      .res_list[tevs->selected_mode]
					      .framerates_size -
				      1];

	if (fps > max_fps)
		fps = max_fps;
	else if (fps < min_fps)
		fps = min_fps;

	fi->interval.numerator = 1;
	fi->interval.denominator = fps;
	tevs->fps = fps;
	dev_dbg(sub_dev->dev, "fps = %d\n", fps);

	return 0;
}

static int tevs_get_frame_desc(struct v4l2_subdev *sub_dev, unsigned int pad,
			       struct v4l2_mbus_frame_desc *fd)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);

	if (pad != 0 || !fd)
		return -EINVAL;

	dev_dbg(sub_dev->dev, "%s(): code [0x%x]\n", __func__, tevs->fmt.code);
	memset(fd, 0x0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->entry[0].flags = 0;
	fd->entry[0].pixelcode = tevs->fmt.code;
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = tevs->vc_id;
	fd->entry[0].bus.csi2.dt =
		tevs->fmt.code == MEDIA_BUS_FMT_SGRBG8_1X8 ?
			MIPI_CSI2_DT_RAW8 :
		tevs->fmt.code == MEDIA_BUS_FMT_SGRBG10_1X10 ?
			MIPI_CSI2_DT_RAW10 :
		tevs->fmt.code == MEDIA_BUS_FMT_SGRBG12_1X12 ?
			MIPI_CSI2_DT_RAW12 :
		tevs->fmt.code == MEDIA_BUS_FMT_SGRBG16_1X16 ?
			MIPI_CSI2_DT_RAW16 :
			MIPI_CSI2_DT_YUV422_8B;
	fd->num_entries = 1;

	return 0;
}

/*
 * V4L2 Controls
 */

static const char *const awb_mode_strings[] = {
	"Manual Temp Mode", // TEVS_AWB_CTRL_MODE_MANUAL_TEMP
	"Auto Mode", // TEVS_AWB_CTRL_MODE_AUTO
	NULL
};

static const char *const flick_mode_strings[] = {
	"Disabled",
	"50 Hz",
	"60 Hz",
	"Auto",
	NULL
};

static const char *const sfx_mode_strings[] = {
	"Normal Mode", // TEVS_SFX_MODE_SFX_NORMAL
	"Black and White Mode", // TEVS_SFX_MODE_SFX_BW
	"Grayscale Mode", // TEVS_SFX_MODE_SFX_GRAYSCALE
	"Negative Mode", // TEVS_SFX_MODE_SFX_NEGATIVE
	"Sketch Mode", // TEVS_SFX_MODE_SFX_SKETCH
	NULL
};

static const char *const ae_mode_strings[] = {
	"Manual Mode", // TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN
	"Auto Mode", // TEVS_AE_CTRL_FULL_AUTO
	"AGC Mode", // TEVS_AE_CTRL_AUTO_GAIN
	NULL
};

static const char *const bsl_mode_strings[] = {
	"Normal Mode",
	"Bootstrap Mode",
	NULL
};

static const char *const trigger_mode_strings[] = {
	"Disabled",
	"Sync to Trigger Mode",
	"Periodic Trigger Mode",
	"Non Periodic Trigger Mode",
	NULL
};

static int tevs_set_brightness(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_BRIGHTNESS,
			 value & TEVS_BRIGHTNESS_MASK, NULL);
}

static int tevs_set_contrast(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_CONTRAST,
			 value & TEVS_CONTRAST_MASK, NULL);
}

static int tevs_set_saturation(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_SATURATION,
			 value & TEVS_SATURATION_MASK, NULL);
}

static int tevs_set_awb_mode(struct tevs *tevs, s32 mode)
{
	u16 val = mode & TEVS_AWB_CTRL_MODE_MASK;

	switch (val) {
	case TEVS_AWB_CTRL_MODE_MANUAL_TEMP_IDX:
		val = TEVS_AWB_CTRL_MODE_MANUAL_TEMP;
		break;
	case TEVS_AWB_CTRL_MODE_AUTO_IDX:
		val = TEVS_AWB_CTRL_MODE_AUTO;
		break;
	default:
		val = TEVS_AWB_CTRL_MODE_AUTO;
		break;
	}

	return cci_write(tevs->regmap, TEVS_AWB_CTRL_MODE, val, NULL);
}

static int tevs_set_gamma(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_GAMMA, value & TEVS_GAMMA_MASK,
			 NULL);
}

static int tevs_set_exposure(struct tevs *tevs, s32 value)
{
	return cci_write(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME,
			 value & TEVS_AE_MANUAL_EXP_TIME_MASK, NULL);
}

static int tevs_set_gain(struct tevs *tevs, s32 value)
{
	return cci_write(tevs->regmap, TEVS_AE_MANUAL_GAIN,
			 value & TEVS_AE_MANUAL_GAIN_MASK, NULL);
}

static int tevs_set_hflip(struct tevs *tevs, s32 flip)
{
	u64 val;
	int ret;

	ret = cci_read(tevs->regmap, TEVS_ORIENTATION, &val, NULL);
	if (ret)
		return ret;

	val &= ~TEVS_ORIENTATION_HFLIP;
	val |= flip ? TEVS_ORIENTATION_HFLIP : 0;

	return cci_write(tevs->regmap, TEVS_ORIENTATION, val, NULL);
}

static int tevs_set_vflip(struct tevs *tevs, s32 flip)
{
	u64 val;
	int ret;

	ret = cci_read(tevs->regmap, TEVS_ORIENTATION, &val, NULL);
	if (ret)
		return ret;

	val &= ~TEVS_ORIENTATION_VFLIP;
	val |= flip ? TEVS_ORIENTATION_VFLIP : 0;

	return cci_write(tevs->regmap, TEVS_ORIENTATION, val, NULL);
}

static int tevs_set_flick_mode(struct tevs *tevs, s32 mode)
{
	u16 val = 0;
	switch (mode) {
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		val = TEVS_FLICK_CTRL_MODE_DISABLED;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		val = TEVS_FLICK_CTRL_MODE_50HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		val = TEVS_FLICK_CTRL_MODE_60HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		val = TEVS_FLICK_CTRL_MODE_AUTO |
		      TEVS_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET |
		      TEVS_FLICK_CTRL_FRC_EN;
		break;
	default:
		val = TEVS_FLICK_CTRL_MODE_DISABLED;
		break;
	}

	return cci_write(tevs->regmap, TEVS_FLICK_CTRL, val, NULL);
}

static int tevs_set_awb_temp(struct tevs *tevs, s32 value)
{
	return cci_write(tevs->regmap, TEVS_AWB_MANUAL_TEMP,
			 value & TEVS_AWB_MANUAL_TEMP_MASK, NULL);
}

static int tevs_set_sharpen(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_SHARPEN, value & TEVS_SHARPEN_MASK,
			 NULL);
}

static int tevs_set_backlight_compensation(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_BACKLIGHT_COMPENSATION,
			 value & TEVS_BACKLIGHT_COMPENSATION_MASK, NULL);
}

static int tevs_set_special_effect(struct tevs *tevs, s32 mode)
{
	u16 val = mode & TEVS_SFX_MODE_SFX_MASK;

	switch (val) {
	case TEVS_SFX_MODE_SFX_NORMAL_IDX:
		val = TEVS_SFX_MODE_SFX_NORMAL;
		break;
	case TEVS_SFX_MODE_SFX_BW_IDX:
		val = TEVS_SFX_MODE_SFX_BW;
		break;
	case TEVS_SFX_MODE_SFX_GRAYSCALE_IDX:
		val = TEVS_SFX_MODE_SFX_GRAYSCALE;
		break;
	case TEVS_SFX_MODE_SFX_NEGATIVE_IDX:
		val = TEVS_SFX_MODE_SFX_NEGATIVE;
		break;
	case TEVS_SFX_MODE_SFX_SKETCH_IDX:
		val = TEVS_SFX_MODE_SFX_SKETCH;
		break;
	default:
		val = TEVS_SFX_MODE_SFX_NORMAL;
		break;
	}

	return cci_write(tevs->regmap, TEVS_SFX_MODE, val, NULL);
}

static int tevs_set_ae_mode(struct tevs *tevs, s32 mode)
{
	u64 val = mode & TEVS_AE_CTRL_MODE_MASK;
	int ret = 0;

	switch (val) {
	case TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN_IDX:
		val = TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN;
		break;
	case TEVS_AE_CTRL_FULL_AUTO_IDX:
		val = TEVS_AE_CTRL_FULL_AUTO;
		break;
	case TEVS_AE_CTRL_AUTO_GAIN_IDX:
		val = TEVS_AE_CTRL_AUTO_GAIN;
		break;
	default:
		val = TEVS_AE_CTRL_FULL_AUTO;
		break;
	}

	ret += cci_write(tevs->regmap, TEVS_AE_CTRL_MODE, val, NULL);
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME, &val, NULL);
	tevs->exp_time->cur.val = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	return ret;
}

static int tevs_set_pan_target(struct tevs *tevs, s32 value)
{
	// Format u7.8
	return cci_write(tevs->regmap, TEVS_DZ_CT_X, value & TEVS_DZ_CT_MASK,
			 NULL);
}

static int tevs_set_tilt_target(struct tevs *tevs, s32 value)
{
	// Format u7.8
	return cci_write(tevs->regmap, TEVS_DZ_CT_Y, value & TEVS_DZ_CT_MASK,
			 NULL);
}

static int tevs_set_zoom_target(struct tevs *tevs, s32 value)
{
	// Format u7.8
	return cci_write(tevs->regmap, TEVS_DZ_TGT_FCT,
			 value & TEVS_DZ_TGT_FCT_MASK, NULL);
}

static int tevs_set_bsl_mode(struct tevs *tevs, s32 mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	u64 val;
	dev_dbg(&client->dev, "%s(): set bls mode: %d", __func__, mode);

	switch (mode) {
	case TEVS_BSL_MODE_NORMAL_IDX:
		gpiod_set_value_cansleep(tevs->reset_gpio, 0);
		usleep_range(9000, 10000);
		gpiod_set_value_cansleep(tevs->reset_gpio, 1);
		usleep_range(9000, 10000);

		msleep(TEVS_BOOT_TIME);

		if (tevs_check_boot_state(tevs) != 0) {
			dev_err(&client->dev,
				"check tevs bootup status failed before change data frequency\n");
			return -ENODEV;
		}

		if (tevs->data_frequency != 0) {
			cci_read(tevs->regmap, HOST_COMMAND_ISP_CTRL_MIPI_FREQ,
				 &val, NULL);
			if (tevs->data_frequency != (val & 0xFFFF)) {
				cci_write(tevs->regmap,
					  HOST_COMMAND_ISP_CTRL_MIPI_FREQ,
					  tevs->data_frequency, NULL);
				msleep(TEVS_BOOT_TIME);
				if (tevs_check_boot_state(tevs) != 0) {
					dev_err(&client->dev,
						"check tevs bootup status failed after change data frequency\n");
					return -ENODEV;
				}
			}
		}

		if (tevs->trigger_mode) {
			switch (tevs->trigger_mode) {
			case TEVS_TRIGGER_MODE_DISABLE_IDX:
				val = TEVS_TRIGGER_MODE_DISABLE;
				break;
			case TEVS_TRIGGER_MODE_SYNC_IDX:
				val = TEVS_TRIGGER_MODE_SYNC;
				break;
			case TEVS_TRIGGER_MODE_PERIODIC_IDX:
				val = TEVS_TRIGGER_MODE_PERIODIC;
				break;
			case TEVS_TRIGGER_MODE_NON_PERIODIC_IDX:
				val = TEVS_TRIGGER_MODE_NON_PERIODIC;
				break;
			default:
				val = TEVS_TRIGGER_MODE_DISABLE;
				break;
			}
			val |= 0x380;
			if (cci_write(tevs->regmap, TEVS_TRIGGER_MODE, val,
				      NULL) != 0) {
				dev_err(&client->dev,
					"set trigger mode failed\n");
				return -EINVAL;
			}
		}

		cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT,
			  TEVS_IMG_FORMAT_UYVY, NULL);
		cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL,
			  0x10 | (tevs->continuous_clock << 5) |
				  (tevs->data_lanes),
			  NULL);
		cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_PREVIEW_MIPI_CTRL,
			  tevs->vc_id, NULL);

		break;
	case TEVS_BSL_MODE_FLASH_IDX:
		gpiod_set_value_cansleep(tevs->reset_gpio, 0);
		usleep_range(9000, 10000);
		gpiod_set_value_cansleep(tevs->standby_gpio, 1);
		msleep(100);
		gpiod_set_value_cansleep(tevs->reset_gpio, 1);
		usleep_range(9000, 10000);
		gpiod_set_value_cansleep(tevs->standby_gpio, 0);
		msleep(100);
		break;
	default:
		dev_err(&client->dev, "%s(): set err bls mode: %d", __func__,
			mode);
		break;
	}

	return 0;
}

static int tevs_set_max_fps(struct tevs *tevs, s32 value)
{
	u64 val;
	int ret = 0;
	ret += cci_write(tevs->regmap, TEVS_MAX_FPS, value & TEVS_MAX_FPS_MASK,
			 NULL);
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME, &val, NULL);
	tevs->exp_time->cur.val = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	return ret;
}

static int tevs_set_denoise(struct tevs *tevs, s32 value)
{
	// Format is u3.12
	return cci_write(tevs->regmap, TEVS_DENOISE, value & TEVS_DENOISE_MASK,
			 NULL);
}

static int tevs_set_ae_auto_exp_upper(struct tevs *tevs, s32 value)
{
	return cci_write(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_UPPER,
			 value & TEVS_AE_AUTO_EXP_TIME_MASK, NULL);
}

static int tevs_set_ae_auto_exp_max(struct tevs *tevs, s32 value)
{
	return cci_write(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_MAX,
			 value & TEVS_AE_AUTO_EXP_TIME_MASK, NULL);
}

static int tevs_set_trigger_mode(struct tevs *tevs, s32 value)
{
	u16 val = value & TEVS_TRIGGER_MODE_MASK;

	switch (val) {
	case TEVS_TRIGGER_MODE_DISABLE_IDX:
		val = TEVS_TRIGGER_MODE_DISABLE;
		break;
	case TEVS_TRIGGER_MODE_SYNC_IDX:
		val = TEVS_TRIGGER_MODE_SYNC;
		break;
	case TEVS_TRIGGER_MODE_PERIODIC_IDX:
		val = TEVS_TRIGGER_MODE_PERIODIC;
		break;
	case TEVS_TRIGGER_MODE_NON_PERIODIC_IDX:
		val = TEVS_TRIGGER_MODE_NON_PERIODIC;
		break;
	default:
		val = TEVS_TRIGGER_MODE_DISABLE;
		break;
	}

	val |= 0x380;
	return cci_write(tevs->regmap, TEVS_TRIGGER_MODE, val, NULL);
}

static int tevs_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tevs *tevs = container_of(ctrl->handler, struct tevs, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return tevs_set_brightness(tevs, ctrl->val);

	case V4L2_CID_CONTRAST:
		return tevs_set_contrast(tevs, ctrl->val);

	case V4L2_CID_SATURATION:
		return tevs_set_saturation(tevs, ctrl->val);

	case V4L2_CID_AUTO_WHITE_BALANCE:
		return tevs_set_awb_mode(tevs, ctrl->val);

	case V4L2_CID_GAMMA:
		return tevs_set_gamma(tevs, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return tevs_set_exposure(tevs, ctrl->val);

	case V4L2_CID_GAIN:
	case V4L2_CID_ANALOGUE_GAIN:
		return tevs_set_gain(tevs, ctrl->val);

	case V4L2_CID_HFLIP:
		return tevs_set_hflip(tevs, ctrl->val);

	case V4L2_CID_VFLIP:
		return tevs_set_vflip(tevs, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return tevs_set_flick_mode(tevs, ctrl->val);

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return tevs_set_awb_temp(tevs, ctrl->val);

	case V4L2_CID_SHARPNESS:
		return tevs_set_sharpen(tevs, ctrl->val);

	case V4L2_CID_BACKLIGHT_COMPENSATION:
		return tevs_set_backlight_compensation(tevs, ctrl->val);

	case V4L2_CID_COLORFX:
		return tevs_set_special_effect(tevs, ctrl->val);

	case V4L2_CID_EXPOSURE_AUTO:
		return tevs_set_ae_mode(tevs, ctrl->val);

	case V4L2_CID_PAN_ABSOLUTE:
		return tevs_set_pan_target(tevs, ctrl->val);

	case V4L2_CID_TILT_ABSOLUTE:
		return tevs_set_tilt_target(tevs, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return tevs_set_zoom_target(tevs, ctrl->val);

	case V4L2_CID_VBLANK:
	case V4L2_CID_HBLANK:
	case V4L2_CID_PIXEL_RATE:
		dev_dbg(&client->dev, "libcamera control 0x%x\n", ctrl->id);
		return 0;

	case V4L2_CID_TEVS_BSL_MODE:
		return tevs_set_bsl_mode(tevs, ctrl->val);

	case V4L2_CID_TEVS_MAX_FPS:
		return tevs_set_max_fps(tevs, ctrl->val);

	case V4L2_CID_TEVS_DENOISE:
		return tevs_set_denoise(tevs, ctrl->val);

	case V4L2_CID_TEVS_AE_EXP_TIME_UPPER:
		return tevs_set_ae_auto_exp_upper(tevs, ctrl->val);

	case V4L2_CID_TEVS_AE_EXP_TIME_MAX:
		return tevs_set_ae_auto_exp_max(tevs, ctrl->val);

	case V4L2_CID_TEVS_TRIGGER_MODE:
		return tevs_set_trigger_mode(tevs, ctrl->val);

	default:
		dev_dbg(&client->dev, "Unknown control 0x%x\n", ctrl->id);
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops tevs_ctrl_ops = {
	.s_ctrl = tevs_s_ctrl,
};

static const struct v4l2_ctrl_config tevs_awb_mode = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_AUTO_WHITE_BALANCE,
	.name = "White_Balance_Mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = TEVS_AWB_CTRL_MODE_AUTO_IDX,
	.def = TEVS_AWB_CTRL_MODE_AUTO_IDX,
	.qmenu = awb_mode_strings,
};

static const struct v4l2_ctrl_config tevs_sfx_mode = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_COLORFX,
	.name = "Special_Effect",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = TEVS_SFX_MODE_SFX_SKETCH_IDX,
	.def = TEVS_SFX_MODE_SFX_NORMAL_IDX,
	.qmenu = sfx_mode_strings,
};

static const struct v4l2_ctrl_config tevs_ae_mode = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_EXPOSURE_AUTO,
	.name = "Exposure_Mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = TEVS_AE_CTRL_AUTO_GAIN_IDX,
	.def = TEVS_AE_CTRL_FULL_AUTO_IDX,
	.qmenu = ae_mode_strings,
};

static const struct v4l2_ctrl_config tevs_bsl_mode = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_BSL_MODE,
	.name = "BSL_Mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = TEVS_BSL_MODE_FLASH_IDX,
	.def = TEVS_BSL_MODE_NORMAL_IDX,
	.qmenu = bsl_mode_strings,
};

static const struct v4l2_ctrl_config tevs_max_fps = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_MAX_FPS,
	.name = "Max_FPS",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x01,
	.max = 0xFF,
	.step = 1,
	.def = 30,
};

static const struct v4l2_ctrl_config tevs_denoise = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_DENOISE,
	.name = "Denoise",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0000,
	.max = 0x4000,
	.step = 1,
	.def = 0x2000,
};

static const struct v4l2_ctrl_config tevs_ae_exp_upper = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_AE_EXP_TIME_UPPER,
	.name = "AE_Exposure_Upper",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0000,
	.max = 0xFFFFFFFFF,
	.step = 1,
	.def = 0x411A,
};

static const struct v4l2_ctrl_config tevs_ae_exp_max = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_AE_EXP_TIME_MAX,
	.name = "AE_Exposure_Max",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0000,
	.max = 0xFFFFFFFFF,
	.step = 1,
	.def = 0x1046A,
};

static const struct v4l2_ctrl_config tevs_trigger_mode = {
	.ops = &tevs_ctrl_ops,
	.id = V4L2_CID_TEVS_TRIGGER_MODE,
	.name = "Trigger_Mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = TEVS_TRIGGER_MODE_NON_PERIODIC_IDX,
	.def = TEVS_TRIGGER_MODE_DISABLE_IDX,
	.qmenu = trigger_mode_strings,
};

static int tevs_ctrls_init(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	u64 val;
	s64 ctrl_def, ctrl_max, ctrl_min;
	static s64 link_freq[] = {
		TEVS_LINK_FREQUENCY_DEFAULT,
	};
	static s64 pixel_rate[] = {
		TEVS_PIXEL_RATE_DEFAULT,
	};

	ctrl_hdlr = &tevs->ctrls;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 29);
	if (ret)
		return ret;

	ret = cci_read(tevs->regmap, TEVS_BRIGHTNESS, &val, NULL);
	ctrl_def = val & TEVS_BRIGHTNESS_MASK;
	ret += cci_read(tevs->regmap, TEVS_BRIGHTNESS_MAX, &val, NULL);
	ctrl_max = val & TEVS_BRIGHTNESS_MASK;
	ret += cci_read(tevs->regmap, TEVS_BRIGHTNESS_MIN, &val, NULL);
	ctrl_min = val & TEVS_BRIGHTNESS_MASK;
	if (ret)
		goto error;
	tevs->brightness = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					     V4L2_CID_BRIGHTNESS, ctrl_min,
					     ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_CONTRAST, &val, NULL);
	ctrl_def = val & TEVS_CONTRAST_MASK;
	ret += cci_read(tevs->regmap, TEVS_CONTRAST_MAX, &val, NULL);
	ctrl_max = val & TEVS_CONTRAST_MASK;
	ret += cci_read(tevs->regmap, TEVS_CONTRAST_MIN, &val, NULL);
	ctrl_min = val & TEVS_CONTRAST_MASK;
	if (ret)
		goto error;
	tevs->contrast = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					   V4L2_CID_CONTRAST, ctrl_min,
					   ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_SATURATION, &val, NULL);
	ctrl_def = val & TEVS_SATURATION_MASK;
	ret += cci_read(tevs->regmap, TEVS_SATURATION_MAX, &val, NULL);
	ctrl_max = val & TEVS_SATURATION_MASK;
	ret += cci_read(tevs->regmap, TEVS_SATURATION_MIN, &val, NULL);
	ctrl_min = val & TEVS_SATURATION_MASK;
	if (ret)
		goto error;
	tevs->saturation = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					     V4L2_CID_SATURATION, ctrl_min,
					     ctrl_max, 1, ctrl_def);

	tevs->awb = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_awb_mode, NULL);
	ret = cci_read(tevs->regmap, TEVS_AWB_CTRL_MODE, &val, NULL);
	if (ret)
		goto error;
	switch (val & TEVS_AWB_CTRL_MODE_MASK) {
	case TEVS_AWB_CTRL_MODE_MANUAL_TEMP:
		tevs->awb->default_value = tevs->awb->cur.val =
			TEVS_AWB_CTRL_MODE_MANUAL_TEMP_IDX;
		break;
	case TEVS_AWB_CTRL_MODE_AUTO:
		tevs->awb->default_value = tevs->awb->cur.val =
			TEVS_AWB_CTRL_MODE_AUTO_IDX;
		break;
	default:
		tevs->awb->default_value = tevs->awb->cur.val =
			TEVS_AWB_CTRL_MODE_AUTO_IDX;
		break;
	}

	ret = cci_read(tevs->regmap, TEVS_GAMMA, &val, NULL);
	ctrl_def = val & TEVS_GAMMA_MASK;
	ret += cci_read(tevs->regmap, TEVS_GAMMA_MAX, &val, NULL);
	ctrl_max = val & TEVS_GAMMA_MASK;
	ret += cci_read(tevs->regmap, TEVS_GAMMA_MIN, &val, NULL);
	ctrl_min = val & TEVS_GAMMA_MASK;
	if (ret)
		goto error;
	tevs->gamma = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					V4L2_CID_GAMMA, ctrl_min, ctrl_max, 1,
					ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME, &val, NULL);
	ctrl_def = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME_MAX, &val, NULL);
	ctrl_max = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME_MIN, &val, NULL);
	ctrl_min = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	if (ret)
		goto error;
	tevs->exp_time = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					   V4L2_CID_EXPOSURE, ctrl_min,
					   ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_AE_MANUAL_GAIN, &val, NULL);
	ctrl_def = val & TEVS_AE_MANUAL_GAIN_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_GAIN_MAX, &val, NULL);
	ctrl_max = val & TEVS_AE_MANUAL_GAIN_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_GAIN_MIN, &val, NULL);
	ctrl_min = val & TEVS_AE_MANUAL_GAIN_MASK;
	if (ret)
		goto error;
	tevs->exp_gain = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					   V4L2_CID_GAIN, ctrl_min, ctrl_max, 1,
					   ctrl_def);
	tevs->alg_gain = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					   V4L2_CID_ANALOGUE_GAIN, ctrl_min,
					   ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_ORIENTATION, &val, NULL);
	ctrl_def = val & TEVS_ORIENTATION_HFLIP;
	if (ret)
		goto error;
	tevs->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					V4L2_CID_HFLIP, 0x0, 0x1, 1, ctrl_def);

	ctrl_def = (val & TEVS_ORIENTATION_VFLIP) >> TEVS_ORIENTATION_VFLIP_BIT;
	tevs->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					V4L2_CID_VFLIP, 0x0, 0x1, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_FLICK_CTRL, &val, NULL);
	if (ret)
		goto error;
	switch (val & TEVS_FLICK_CTRL_MODE_MASK) {
	case TEVS_FLICK_CTRL_MODE_DISABLED:
		ctrl_def = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
		break;
	case TEVS_FLICK_CTRL_MODE_MANUAL:
		if ((val & TEVS_FLICK_CTRL_FREQ_MASK) ==
		    TEVS_FLICK_CTRL_FREQ(50))
			ctrl_def = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
		else if ((val & TEVS_FLICK_CTRL_FREQ_MASK) ==
			 TEVS_FLICK_CTRL_FREQ(60))
			ctrl_def = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
		break;
	case TEVS_FLICK_CTRL_MODE_AUTO:
		ctrl_def = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
		break;
	default:
		ctrl_def = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
		break;
	}
	tevs->flick = v4l2_ctrl_new_std_menu(ctrl_hdlr, &tevs_ctrl_ops,
					     V4L2_CID_POWER_LINE_FREQUENCY,
					     V4L2_CID_POWER_LINE_FREQUENCY_AUTO,
					     0, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_AWB_MANUAL_TEMP, &val, NULL);
	ctrl_def = val & TEVS_AWB_MANUAL_TEMP_MASK;
	ret += cci_read(tevs->regmap, TEVS_AWB_MANUAL_TEMP_MAX, &val, NULL);
	ctrl_max = val & TEVS_AWB_MANUAL_TEMP_MASK;
	ret += cci_read(tevs->regmap, TEVS_AWB_MANUAL_TEMP_MIN, &val, NULL);
	ctrl_min = val & TEVS_AWB_MANUAL_TEMP_MASK;
	if (ret)
		goto error;
	tevs->wb_temp = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					  V4L2_CID_WHITE_BALANCE_TEMPERATURE,
					  ctrl_min, ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_SHARPEN, &val, NULL);
	ctrl_def = val & TEVS_SHARPEN_MASK;
	ret += cci_read(tevs->regmap, TEVS_SHARPEN_MAX, &val, NULL);
	ctrl_max = val & TEVS_SHARPEN_MASK;
	ret += cci_read(tevs->regmap, TEVS_SHARPEN_MIN, &val, NULL);
	ctrl_min = val & TEVS_SHARPEN_MASK;
	if (ret)
		goto error;
	tevs->sharpness = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					    V4L2_CID_SHARPNESS, ctrl_min,
					    ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_BACKLIGHT_COMPENSATION, &val, NULL);
	ctrl_def = val & TEVS_BACKLIGHT_COMPENSATION_MASK;
	ret += cci_read(tevs->regmap, TEVS_BACKLIGHT_COMPENSATION_MAX, &val,
			NULL);
	ctrl_max = val & TEVS_BACKLIGHT_COMPENSATION_MASK;
	ret += cci_read(tevs->regmap, TEVS_BACKLIGHT_COMPENSATION_MIN, &val,
			NULL);
	ctrl_min = val & TEVS_BACKLIGHT_COMPENSATION_MASK;
	if (ret)
		goto error;
	tevs->backlight_comp = v4l2_ctrl_new_std(
		ctrl_hdlr, &tevs_ctrl_ops, V4L2_CID_BACKLIGHT_COMPENSATION,
		ctrl_min, ctrl_max, 1, ctrl_def);

	tevs->colorfx = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_sfx_mode, NULL);
	ret = cci_read(tevs->regmap, TEVS_SFX_MODE, &val, NULL);
	if (ret)
		goto error;
	switch (val & TEVS_SFX_MODE_SFX_MASK) {
	case TEVS_SFX_MODE_SFX_NORMAL:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_NORMAL_IDX;
		break;
	case TEVS_SFX_MODE_SFX_BW:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_BW_IDX;
		break;
	case TEVS_SFX_MODE_SFX_GRAYSCALE:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_GRAYSCALE_IDX;
		break;
	case TEVS_SFX_MODE_SFX_NEGATIVE:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_NEGATIVE_IDX;
		break;
	case TEVS_SFX_MODE_SFX_SKETCH:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_SKETCH_IDX;
		break;
	default:
		tevs->colorfx->default_value = tevs->colorfx->cur.val =
			TEVS_SFX_MODE_SFX_NORMAL_IDX;
		break;
	}

	tevs->ae = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_ae_mode, NULL);
	ret = cci_read(tevs->regmap, TEVS_AE_CTRL_MODE, &val, NULL);
	if (ret)
		goto error;
	switch (val & TEVS_AE_CTRL_MODE_MASK) {
	case TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN:
		tevs->ae->default_value = tevs->ae->cur.val =
			TEVS_AE_CTRL_MANUAL_EXP_TIME_GAIN_IDX;
		break;
	case TEVS_AE_CTRL_FULL_AUTO:
		tevs->ae->default_value = tevs->ae->cur.val =
			TEVS_AE_CTRL_FULL_AUTO_IDX;
		break;
	case TEVS_AE_CTRL_AUTO_GAIN: {
		ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME, &val,
				NULL);
		tevs->exp_time->cur.val = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
		tevs->ae->default_value = tevs->ae->cur.val =
			TEVS_AE_CTRL_AUTO_GAIN_IDX;
	} break;
	default:
		tevs->ae->default_value = tevs->ae->cur.val =
			TEVS_AE_CTRL_FULL_AUTO_IDX;
		break;
	}

	ret = cci_read(tevs->regmap, TEVS_DZ_CT_X, &val, NULL);
	ctrl_def = val & TEVS_DZ_CT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_CT_MAX, &val, NULL);
	ctrl_max = val & TEVS_DZ_CT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_CT_MIN, &val, NULL);
	ctrl_min = val & TEVS_DZ_CT_MASK;
	if (ret)
		goto error;
	tevs->pan = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
				      V4L2_CID_PAN_ABSOLUTE, ctrl_min, ctrl_max,
				      1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_DZ_CT_Y, &val, NULL);
	ctrl_def = val & TEVS_DZ_CT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_CT_MAX, &val, NULL);
	ctrl_max = val & TEVS_DZ_CT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_CT_MIN, &val, NULL);
	ctrl_min = val & TEVS_DZ_CT_MASK;
	if (ret)
		goto error;
	tevs->tilt = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
				       V4L2_CID_TILT_ABSOLUTE, ctrl_min,
				       ctrl_max, 1, ctrl_def);

	ret = cci_read(tevs->regmap, TEVS_DZ_TGT_FCT, &val, NULL);
	ctrl_def = val & TEVS_DZ_TGT_FCT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_TGT_FCT_MAX, &val, NULL);
	ctrl_max = val & TEVS_DZ_TGT_FCT_MASK;
	ret += cci_read(tevs->regmap, TEVS_DZ_TGT_FCT_MIN, &val, NULL);
	ctrl_min = val & TEVS_DZ_TGT_FCT_MASK;
	if (ret)
		goto error;
	tevs->zoom = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
				       V4L2_CID_ZOOM_ABSOLUTE, ctrl_min,
				       ctrl_max, 1, ctrl_def);

	tevs->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					 V4L2_CID_HBLANK, 0, 0, 1, 0);
	tevs->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					 V4L2_CID_VBLANK, 0, 0, 1, 0);

	/* By default, link_freq and pixel_rate is read only */
	link_freq[0] = (u64)(tevs->data_frequency >> 1) * 1000000ULL;
	tevs->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &tevs_ctrl_ops,
						 V4L2_CID_LINK_FREQ,
						 ARRAY_SIZE(link_freq) - 1, 0,
						 link_freq);
	tevs->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* link_freq = (pixel_rate * bpp) / (2 * data_lanes) */
	pixel_rate[0] = div_s64(link_freq[0] * (2 * tevs->data_lanes), 16);
	tevs->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &tevs_ctrl_ops,
					     V4L2_CID_PIXEL_RATE, pixel_rate[0],
					     pixel_rate[0], 1, pixel_rate[0]);
	tevs->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	tevs->bsl = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_bsl_mode, NULL);

	tevs->max_fps = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_max_fps, NULL);
	ret = cci_read(tevs->regmap, TEVS_MAX_FPS, &val, NULL);
	if (ret)
		goto error;
	tevs->max_fps->default_value = tevs->max_fps->cur.val =
		val & TEVS_MAX_FPS_MASK;

	tevs->denoise = v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_denoise, NULL);
	ret = cci_read(tevs->regmap, TEVS_DENOISE, &val, NULL);
	ctrl_def = val & TEVS_DENOISE_MASK;
	ret += cci_read(tevs->regmap, TEVS_DENOISE_MAX, &val, NULL);
	ctrl_max = val & TEVS_DENOISE_MASK;
	ret += cci_read(tevs->regmap, TEVS_DENOISE_MIN, &val, NULL);
	ctrl_min = val & TEVS_DENOISE_MASK;
	if (ret)
		goto error;
	tevs->denoise->default_value = tevs->denoise->cur.val = ctrl_def;
	tevs->denoise->maximum = ctrl_max;
	tevs->denoise->minimum = ctrl_min;

	tevs->ae_exp_upper =
		v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_ae_exp_upper, NULL);
	ret = cci_read(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_UPPER, &val, NULL);
	ctrl_def = val & TEVS_AE_AUTO_EXP_TIME_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME_MAX, &val, NULL);
	ctrl_max = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	ret += cci_read(tevs->regmap, TEVS_AE_MANUAL_EXP_TIME_MIN, &val, NULL);
	ctrl_min = val & TEVS_AE_MANUAL_EXP_TIME_MASK;
	if (ret)
		goto error;
	tevs->ae_exp_upper->default_value = tevs->ae_exp_upper->cur.val =
		ctrl_def;
	tevs->ae_exp_upper->maximum = ctrl_max;
	tevs->ae_exp_upper->minimum = ctrl_min;

	tevs->ae_exp_max =
		v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_ae_exp_max, NULL);
	ret = cci_read(tevs->regmap, TEVS_AE_AUTO_EXP_TIME_MAX, &val, NULL);
	ctrl_def = val & TEVS_AE_AUTO_EXP_TIME_MASK;
	if (ret)
		goto error;
	tevs->ae_exp_max->default_value = tevs->ae_exp_max->cur.val = ctrl_def;
	tevs->ae_exp_max->maximum = ctrl_max;
	tevs->ae_exp_max->minimum = ctrl_min;

	tevs->trigger =
		v4l2_ctrl_new_custom(ctrl_hdlr, &tevs_trigger_mode, NULL);
	tevs->trigger->default_value = tevs->trigger->cur.val =
		tevs->trigger_mode;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "ctrls init error (%d)\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &tevs_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	tevs->v4l2_subdev.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void tevs_ctrls_free(struct tevs *tevs)
{
	v4l2_ctrl_handler_free(&tevs->ctrls);
}

static int tevs_media_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int tevs_power_on(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	int ret = 0;
	dev_dbg(&client->dev, "%s()\n", __func__);

	gpiod_set_value_cansleep(tevs->host_pwdn_gpio, 1);
	gpiod_set_value_cansleep(tevs->reset_gpio, 1);
	msleep(TEVS_BOOT_TIME);

	ret = tevs_check_boot_state(tevs);
	if (ret != 0) {
		goto error;
	}

	if (tevs->trigger_mode | tevs->hw_reset_mode) {
		ret = tevs_set_trigger_mode(tevs, tevs->trigger_mode);
		if (ret != 0) {
			dev_err(&client->dev, "set trigger mode failed\n");
			return ret;
		}

		ret += cci_write(tevs->regmap,
				 HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT,
				 TEVS_IMG_FORMAT_UYVY, NULL);
		ret += cci_write(tevs->regmap,
				 HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL,
				 0x10 | (tevs->continuous_clock << 5) |
					 (tevs->data_lanes),
				 NULL);
		ret += cci_write(tevs->regmap,
				 HOST_COMMAND_ISP_CTRL_PREVIEW_MIPI_CTRL,
				 tevs->vc_id, NULL);
	}

	return ret;

error:
	gpiod_set_value_cansleep(tevs->reset_gpio, 0);
	gpiod_set_value_cansleep(tevs->host_pwdn_gpio, 0);
	return ret;
}

static int tevs_power_off(struct tevs *tevs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&tevs->v4l2_subdev);
	dev_dbg(&client->dev, "%s()\n", __func__);

	if (tevs->hw_reset_mode) {
		gpiod_set_value_cansleep(tevs->reset_gpio, 0);
		gpiod_set_value_cansleep(tevs->standby_gpio, 0);
		gpiod_set_value_cansleep(tevs->host_pwdn_gpio, 0);
	}

	return 0;
}

static int tevs_power(struct v4l2_subdev *sub_dev, int on)
{
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);
	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, on);
	if (on)
		return tevs_power_on(tevs);
	else
		return tevs_power_off(tevs);
}

static const struct v4l2_subdev_core_ops tevs_v4l2_subdev_core_ops = {
	// s_power only for staging isi driver
	.s_power = tevs_power,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tevs_v4l2_subdev_video_ops = {
	.s_stream = tevs_set_stream,
};

static const struct v4l2_subdev_pad_ops tevs_v4l2_subdev_pad_ops = {
	.enum_mbus_code = tevs_enum_mbus_code,
	.enum_frame_size = tevs_enum_frame_size,
	.enum_frame_interval = tevs_enum_frame_interval,
	.get_fmt = tevs_get_fmt,
	.set_fmt = tevs_set_fmt,
	.get_selection = tevs_get_selection,
	.get_frame_interval = tevs_get_frame_interval,
	.set_frame_interval = tevs_set_frame_interval,
	.get_frame_desc = tevs_get_frame_desc,
};

static const struct v4l2_subdev_ops tevs_subdev_ops = {
	.core = &tevs_v4l2_subdev_core_ops,
	.video = &tevs_v4l2_subdev_video_ops,
	.pad = &tevs_v4l2_subdev_pad_ops,
};

static const struct media_entity_operations tevs_media_entity_ops = {
	.link_setup = tevs_media_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static int tevs_try_on(struct tevs *tevs)
{
	tevs_power_off(tevs);
	return tevs_power_on(tevs);
}

static int tevs_check_hwcfg(struct device *dev, struct tevs *tevs)
{
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint ep_cfg = { .bus_type =
						       V4L2_MBUS_CSI2_DPHY };
	int ret = -EINVAL;

	tevs->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tevs->reset_gpio)) {
		ret = PTR_ERR(tevs->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get reset GPIO (%d)", ret);
		return ret;
	}

	tevs->host_pwdn_gpio =
		devm_gpiod_get_optional(dev, "host-pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(tevs->host_pwdn_gpio)) {
		ret = PTR_ERR(tevs->host_pwdn_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get host-pwdn GPIO (%d)", ret);
		return ret;
	}

	tevs->standby_gpio =
		devm_gpiod_get_optional(dev, "standby", GPIOD_OUT_LOW);
	if (IS_ERR(tevs->standby_gpio)) {
		ret = PTR_ERR(tevs->standby_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get standby GPIO (%d)", ret);
		return ret;
	}
	gpiod_set_value_cansleep(tevs->standby_gpio, 0);

	tevs->supports_over_4k_res =
		of_property_read_bool(dev->of_node, "supports-over-4k-res");

	tevs->vc_id = 0;
	if (of_property_read_u32(dev->of_node, "vc-id", &tevs->vc_id) == 0) {
		if (tevs->vc_id > 3) {
			dev_err(dev,
				"value of 'vc-id = <%d>' property is invaild\n",
				tevs->vc_id);
			return ret;
		}
	}

	tevs->hw_reset_mode = of_property_read_bool(dev->of_node, "hw-reset");

	tevs->trigger_mode = 0;
	if (of_property_read_u32(dev->of_node, "trigger-mode",
				 &tevs->trigger_mode) == 0) {
		if (tevs->trigger_mode > 3) {
			dev_err(dev,
				"value of 'trigger-mode = <%d>' property is invaild\n",
				tevs->trigger_mode);
			return ret;
		}
	}

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep) {
		dev_err(dev, "no sink port found");
		return ret;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ep_cfg);
	if (ret < 0) {
		dev_err(dev, "failed to parse bus configuration\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev,
			"only 2 or 4 data lanes are currently supported\n");
		goto error_out;
	}
	tevs->data_lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;

	/* Check the link frequency set in device tree */
	if (ep_cfg.nr_of_link_frequencies == 0)
		tevs->data_frequency =
			(u32)div_u64(TEVS_LINK_FREQUENCY_DEFAULT, 1000000ULL) *
			2;
	else if (ep_cfg.nr_of_link_frequencies == 1)
		tevs->data_frequency =
			(u32)div_u64(ep_cfg.link_frequencies[0], 1000000ULL) *
			2;
	else {
		dev_err(dev, "invalid link frequencies %u on port\n",
			ep_cfg.nr_of_link_frequencies);
		goto error_out;
	}

	if ((tevs->data_frequency != 0) &&
	    ((tevs->data_frequency < 100) || (tevs->data_frequency > 1200))) {
		dev_err(dev, "value of data-frequency [%d] is invaild\n",
			tevs->data_frequency);
		goto error_out;
	}

	tevs->continuous_clock = !(ep_cfg.bus.mipi_csi2.flags &
				 V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK);

	dev_dbg(dev,
		"data-lanes [%d], continuous-clock [%d], supports-over-4k-res [%d],"
		" vc-id [%d], hw-reset [%d], trigger-mode [%d]\n",
		tevs->data_lanes, tevs->continuous_clock,
		tevs->supports_over_4k_res, tevs->vc_id, tevs->hw_reset_mode,
		tevs->trigger_mode);

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(ep);

	return ret;
}

static int tevs_probe(struct i2c_client *client)
{
	struct tevs *tevs = NULL;
	struct device *dev = &client->dev;
	struct v4l2_mbus_framefmt *fmt;
	int i = ARRAY_SIZE(tevs_sensor_table);
	int ret;

	dev_info(dev, "%s() device node: %s\n", __func__,
		 client->dev.of_node->full_name);

	tevs = devm_kzalloc(dev, sizeof(struct tevs), GFP_KERNEL);
	if (tevs == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&tevs->v4l2_subdev, client, &tevs_subdev_ops);

	i2c_set_clientdata(client, tevs);
	tevs->regmap = devm_regmap_init_i2c(client, &tevs_regmap_config);
	if (IS_ERR(tevs->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	ret = tevs_check_hwcfg(dev, tevs);
	if (ret < 0)
		return ret;

	if (tevs_try_on(tevs) != 0) {
		dev_err(dev, "cannot find tevs camera\n");
		return -ENODEV;
	}

	if (tevs->data_frequency != 0) {
		ret = cci_write(tevs->regmap, HOST_COMMAND_ISP_CTRL_MIPI_FREQ,
				tevs->data_frequency, NULL);
		msleep(TEVS_BOOT_TIME);
		if (tevs_check_boot_state(tevs) != 0) {
			dev_err(dev, "check tevs bootup status failed\n");
			ret = -ENODEV;
			goto error_power_off;
		}
		if (ret < 0) {
			dev_err(dev, "set mipi frequency failed\n");
			goto error_power_off;
		}
	}

	ret = tevs_check_version(tevs);
	if (ret < 0) {
		dev_err(dev, "check device version failed\n");
		goto error_power_off;
	}

	tevs->header_info =
		devm_kzalloc(dev, sizeof(struct header_info), GFP_KERNEL);
	if (tevs->header_info == NULL) {
		dev_err(dev, "allocate header_info failed\n");
		ret = -ENOMEM;
		goto error_power_off;
	}

	ret = tevs_load_header_info(tevs);
	if (ret < 0) {
		dev_err(dev, "load header information failed\n");
		goto error_power_off;
	}

	ret = tevs_get_chip_id(tevs);

	if (ret < 0) {
		dev_err(dev, "get chip ID failed\n");
		goto error_power_off;
	}

	if (tevs->chip_id == SENSOR_CHIP_ID_NONE) {
		for (i = 0; i < ARRAY_SIZE(tevs_sensor_table); i++) {
			if (strcmp((const char *)tevs->header_info->product_name,
				   tevs_sensor_table[i].sensor_name) == 0)
				break;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(tevs_sensor_table); i++) {
			if (tevs->chip_id == tevs_sensor_table[i].chip_id)
				break;
		}
	}

	if (i >= ARRAY_SIZE(tevs_sensor_table)) {
		if (tevs->chip_id == SENSOR_CHIP_ID_NONE)
			dev_err(dev, "cannot not support the product: %s\n",
				(const char *)tevs->header_info->product_name);
		else
			dev_err(dev, "cannot not support the chip ID: 0x%.4X\n",
				tevs->chip_id);

		ret = -ENODEV;
		goto error_power_off;
	}

	tevs->selected_sensor = i;
	dev_dbg(dev, "selected_sensor:%d, sensor_name:%s\n", i,
		tevs->header_info->product_name);

	/* Initialize default format */
	fmt = &tevs->fmt;
	fmt->width = tevs_sensor_table[tevs->selected_sensor].res_list[0].width;
	fmt->height =
		tevs_sensor_table[tevs->selected_sensor].res_list[0].height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = tevs_sensor_table[tevs->selected_sensor].code_list[0];
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	ret = tevs_ctrls_init(tevs);
	if (ret) {
		dev_err(dev, "failed to init controls: %d", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	tevs->v4l2_subdev.flags |=
		(V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE);
	tevs->v4l2_subdev.entity.ops = &tevs_media_entity_ops;
	tevs->v4l2_subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	tevs->pad.flags = MEDIA_PAD_FL_SOURCE;
	tevs->fps = tevs_sensor_table[tevs->selected_sensor]
			    .res_list[0]
			    .framerates[0];
	ret = media_entity_pads_init(&tevs->v4l2_subdev.entity, 1, &tevs->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&tevs->v4l2_subdev);
	if (ret) {
		dev_err(dev, "failed to initialize subdev state: %d\n", ret);
		goto error_subdev_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&tevs->v4l2_subdev);
	if (ret != 0) {
		dev_err(dev, "v4l2 register failed\n");
		goto error_media_entity;
	}

	if (tevs->trigger_mode) {
		ret = tevs_set_trigger_mode(tevs, tevs->trigger_mode);
		if (ret != 0) {
			dev_err(dev, "set trigger mode failed\n");
			goto error_media_entity;
		}
	}

	if (!(tevs->hw_reset_mode | tevs_check_trigger_mode(tevs))) {
		ret = tevs_standby(tevs, 1);
		if (ret != 0) {
			dev_err(dev, "set standby mode failed\n");
			goto error_media_entity;
		}
		if (tevs->continuous_clock) {
			cci_write(tevs->regmap,
				  HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL,
				  0x10 | (TEVS_CONTINUOUS_CLOCK_DEFAULT  << 5) |
					  (tevs->data_lanes),
				  NULL);
		}
	} else {
		ret = tevs_power_off(tevs);
		if (ret != 0) {
			dev_err(dev, "set power off failed\n");
			goto error_media_entity;
		}
	}

	dev_info(dev, "probe success\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&tevs->v4l2_subdev.entity);

error_subdev_cleanup:
	v4l2_subdev_cleanup(&tevs->v4l2_subdev);

error_handler_free:
	tevs_ctrls_free(tevs);

error_power_off:
	tevs_power_off(tevs);

	dev_err(dev, "probe failed\n");
	return ret;
}

static void tevs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sub_dev = i2c_get_clientdata(client);
	struct tevs *tevs = container_of(sub_dev, struct tevs, v4l2_subdev);

	v4l2_async_unregister_subdev(sub_dev);
	media_entity_cleanup(&sub_dev->entity);
	v4l2_subdev_cleanup(&tevs->v4l2_subdev);
	tevs_ctrls_free(tevs);
}

static const struct of_device_id sensor_of[] = {
	{ .compatible = "tn,tevs" },
	{ .compatible = "tn,tevm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sensor_of);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name  = "tevs",
		.of_match_table = of_match_ptr(sensor_of),
	},
	.probe = tevs_probe,
	.remove = tevs_remove,
};

module_i2c_driver(sensor_i2c_driver);

MODULE_AUTHOR("TECHNEXION Inc.");
MODULE_DESCRIPTION("TechNexion TEVS camera driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
MODULE_ALIAS("Camera");
