#ifndef BMP380_H_
#define BMP380_H_

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/regmap.h>

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)	S8_C(x)
#define UINT8_C(x)	U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)	S16_C(x)
#define UINT16_C(x)	U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)	S32_C(x)
#define UINT32_C(x)	U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)	S64_C(x)
#define UINT64_C(x)	U64_C(x)
#endif

#define BMP3_I2C_ADDR_PRIM			UINT8_C(0x76)
#define BMP3_I2C_ADDR_SEC			UINT8_C(0x77)

#define BMP3_CHIP_ID				UINT8_C(0x50)

#define BMP3_PRESS_SETTLE_TIME			UINT16_C(392)

#define BMP3_TEMP_SETTLE_TIME			UINT16_C(313)

#define BMP3_ADC_CONV_TIME			UINT16_C(2000)

#define BMP3_CHIP_ID_ADDR			0x00
#define BMP3_ERR_REG_ADDR			0x02
#define BMP3_SENS_STATUS_REG_ADDR		0x03
#define BMP3_PRESS_ADDR				0x04
#define BMP3_TEMPE_ADDR				0x07
#define BMP3_EVENT_ADDR				0x10
#define BMP3_INT_STATUS_REG_ADDR		0x11
#define BMP3_FIFO_LENGTH_ADDR			0x12
#define BMP3_FIFO_DATA_ADDR			0x14
#define BMP3_FIFO_WM_ADDR			0x15
#define BMP3_FIFO_CONFIG_1_ADDR			0x17
#define BMP3_FIFO_CONFIG_2_ADDR			0x18
#define BMP3_INT_CTRL_ADDR			0x19
#define BMP3_IF_CONF_ADDR			0x1A
#define BMP3_PWR_CTRL_ADDR			0x1B
#define BMP3_OSR_ADDR				0X1C
#define BMP3_CALIB_DATA_ADDR			0x31
#define BMP3_CMD_ADDR				0x7E
#define BMP380_CMD_SOFTRESET			0xB6

#define BMP3_FATAL_ERR				0x01
#define BMP3_CMD_ERR				0x02
#define BMP3_CONF_ERR				0x04
#define BMP3_CMD_RDY				0x10
#define BMP3_CMD_PARSE				0x13
#define BMP3_DRDY_PRESS				0x20
#define BMP3_DRDY_TEMP				0x40

#define BMP3_SLEEP_MODE				0x00
#define BMP3_FORCED_MODE			0x01
#define BMP3_NORMAL_MODE			0x03

#define BMP3_ENABLE				0x01
#define BMP3_DISABLE				0x00

#define BMP3_INT_PIN_OPEN_DRAIN			0x01
#define BMP3_INT_PIN_PUSH_PULL			0x00

#define BMP3_INT_PIN_ACTIVE_HIGH		0x01
#define BMP3_INT_PIN_ACTIVE_LOW			0x00

#define BMP3_INT_PIN_LATCH			0x01
#define BMP3_INT_PIN_NON_LATCH			0x00

#define BMP3_I2C_WDT_SHORT_1_25_MS		0x00
#define BMP3_I2C_WDT_LONG_40_MS			0x01

#define BMP3_FIFO_NO_SUBSAMPLING		0x00
#define BMP3_FIFO_SUBSAMPLING_2X		0x01
#define BMP3_FIFO_SUBSAMPLING_4X		0x02
#define BMP3_FIFO_SUBSAMPLING_8X		0x03
#define BMP3_FIFO_SUBSAMPLING_16X		0x04
#define BMP3_FIFO_SUBSAMPLING_32X		0x05
#define BMP3_FIFO_SUBSAMPLING_64X		0x06
#define BMP3_FIFO_SUBSAMPLING_128X		0x07

#define BMP3_NO_OVERSAMPLING			0x00
#define BMP3_OVERSAMPLING_2X			0x01
#define BMP3_OVERSAMPLING_4X			0x02
#define BMP3_OVERSAMPLING_8X			0x03
#define BMP3_OVERSAMPLING_16X			0x04
#define BMP3_OVERSAMPLING_32X			0x05

#define BMP3_IIR_FILTER_DISABLE			0x00
#define BMP3_IIR_FILTER_COEFF_1			0x01
#define BMP3_IIR_FILTER_COEFF_3			0x02
#define BMP3_IIR_FILTER_COEFF_7			0x03
#define BMP3_IIR_FILTER_COEFF_15		0x04
#define BMP3_IIR_FILTER_COEFF_31		0x05
#define BMP3_IIR_FILTER_COEFF_63		0x06
#define BMP3_IIR_FILTER_COEFF_127		0x07

#define BMP3_ODR_200_HZ				0x00
#define BMP3_ODR_100_HZ				0x01
#define BMP3_ODR_50_HZ				0x02
#define BMP3_ODR_25_HZ				0x03
#define BMP3_ODR_12_5_HZ			0x04
#define BMP3_ODR_6_25_HZ			0x05
#define BMP3_ODR_3_1_HZ				0x06
#define BMP3_ODR_1_5_HZ				0x07
#define BMP3_ODR_0_78_HZ			0x08
#define BMP3_ODR_0_39_HZ			0x09
#define BMP3_ODR_0_2_HZ				0x0A
#define BMP3_ODR_0_1_HZ				0x0B
#define BMP3_ODR_0_05_HZ			0x0C
#define BMP3_ODR_0_02_HZ			0x0D
#define BMP3_ODR_0_01_HZ			0x0E
#define BMP3_ODR_0_006_HZ			0x0F
#define BMP3_ODR_0_003_HZ			0x10
#define BMP3_ODR_0_001_HZ			0x11

#define BMP3_OK					INT8_C(0)

#define BMP3_E_NULL_PTR				INT8_C(-1)
#define BMP3_E_DEV_NOT_FOUND			INT8_C(-2)
#define BMP3_E_INVALID_ODR_OSR_SETTINGS		INT8_C(-3)
#define BMP3_E_CMD_EXEC_FAILED			INT8_C(-4)
#define BMP3_E_CONFIGURATION_ERR		INT8_C(-5)
#define BMP3_E_INVALID_LEN			INT8_C(-6)
#define BMP3_E_COMM_FAIL			INT8_C(-7)
#define BMP3_E_FIFO_WATERMARK_NOT_REACHED	INT8_C(-8)

#define BMP3_W_SENSOR_NOT_ENABLED		UINT8_C(1)
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT	UINT8_C(2)

#define	BMP3_PRESS_EN_SEL			UINT16_C(1 << 1)
#define BMP3_TEMP_EN_SEL			UINT16_C(1 << 2)
#define BMP3_DRDY_EN_SEL			UINT16_C(1 << 3)
#define BMP3_PRESS_OS_SEL			UINT16_C(1 << 4)
#define BMP3_TEMP_OS_SEL			UINT16_C(1 << 5)
#define BMP3_IIR_FILTER_SEL			UINT16_C(1 << 6)
#define BMP3_ODR_SEL				UINT16_C(1 << 7)
#define BMP3_OUTPUT_MODE_SEL			UINT16_C(1 << 8)
#define BMP3_LEVEL_SEL				UINT16_C(1 << 9)
#define BMP3_LATCH_SEL				UINT16_C(1 << 10)
#define BMP3_I2C_WDT_EN_SEL			UINT16_C(1 << 11)
#define BMP3_I2C_WDT_SEL_SEL			UINT16_C(1 << 12)
#define BMP3_ALL_SETTINGS			UINT16_C(0x7FF)

#define BMP3_FIFO_MODE_SEL			UINT16_C(1 << 1)
#define BMP3_FIFO_STOP_ON_FULL_EN_SEL		UINT16_C(1 << 2)
#define BMP3_FIFO_TIME_EN_SEL			UINT16_C(1 << 3)
#define BMP3_FIFO_PRESS_EN_SEL			UINT16_C(1 << 4)
#define BMP3_FIFO_TEMP_EN_SEL			UINT16_C(1 << 5)
#define BMP3_FIFO_DOWN_SAMPLING_SEL		UINT16_C(1 << 6)
#define BMP3_FIFO_FILTER_EN_SEL			UINT16_C(1 << 7)
#define BMP3_FIFO_FWTM_EN_SEL			UINT16_C(1 << 8)
#define BMP3_FIFO_FULL_EN_SEL			UINT16_C(1 << 9)
#define BMP3_FIFO_ALL_SETTINGS			UINT16_C(0x3FF)

#define BMP3_PRESS        			UINT8_C(1)
#define BMP3_TEMP         			UINT8_C(1 << 1)
#define BMP3_ALL          			UINT8_C(0x03)

#define BMP3_ERR_FATAL_MSK			UINT8_C(0x01)

#define BMP3_ERR_CMD_MSK			UINT8_C(0x02)
#define BMP3_ERR_CMD_POS			UINT8_C(0x01)

#define BMP3_ERR_CONF_MSK			UINT8_C(0x04)
#define BMP3_ERR_CONF_POS			UINT8_C(0x02)

#define BMP3_STATUS_CMD_RDY_MSK			UINT8_C(0x10)
#define BMP3_STATUS_CMD_RDY_POS			UINT8_C(0x04)

#define BMP3_STATUS_DRDY_PRESS_MSK		UINT8_C(0x20)
#define BMP3_STATUS_DRDY_PRESS_POS		UINT8_C(0x05)

#define BMP3_STATUS_DRDY_TEMP_MSK		UINT8_C(0x40)
#define BMP3_STATUS_DRDY_TEMP_POS		UINT8_C(0x06)

#define BMP3_OP_MODE_MSK			UINT8_C(0x30)
#define BMP3_OP_MODE_POS			UINT8_C(0x04)

#define BMP3_PRESS_EN_MSK			UINT8_C(0x01)

#define BMP3_TEMP_EN_MSK			UINT8_C(0x02)
#define BMP3_TEMP_EN_POS			UINT8_C(0x01)

#define BMP3_IIR_FILTER_MSK			UINT8_C(0x0E)
#define BMP3_IIR_FILTER_POS			UINT8_C(0x01)

#define BMP3_ODR_MSK				UINT8_C(0x1F)

#define BMP3_PRESS_OS_MSK			UINT8_C(0x07)

#define BMP3_TEMP_OS_MSK			UINT8_C(0x38)
#define BMP3_TEMP_OS_POS			UINT8_C(0x03)

#define BMP3_FIFO_MODE_MSK			UINT8_C(0x01)

#define BMP3_FIFO_STOP_ON_FULL_MSK		UINT8_C(0x02)
#define BMP3_FIFO_STOP_ON_FULL_POS		UINT8_C(0x01)

#define BMP3_FIFO_TIME_EN_MSK			UINT8_C(0x04)
#define BMP3_FIFO_TIME_EN_POS			UINT8_C(0x02)

#define BMP3_FIFO_PRESS_EN_MSK			UINT8_C(0x08)
#define BMP3_FIFO_PRESS_EN_POS			UINT8_C(0x03)

#define BMP3_FIFO_TEMP_EN_MSK			UINT8_C(0x10)
#define BMP3_FIFO_TEMP_EN_POS			UINT8_C(0x04)

#define BMP3_FIFO_FILTER_EN_MSK			UINT8_C(0x18)
#define BMP3_FIFO_FILTER_EN_POS			UINT8_C(0x03)

#define BMP3_FIFO_DOWN_SAMPLING_MSK		UINT8_C(0x07)

#define BMP3_FIFO_FWTM_EN_MSK			UINT8_C(0x08)
#define BMP3_FIFO_FWTM_EN_POS			UINT8_C(0x03)

#define BMP3_FIFO_FULL_EN_MSK			UINT8_C(0x10)
#define BMP3_FIFO_FULL_EN_POS			UINT8_C(0x04)

#define BMP3_INT_OUTPUT_MODE_MSK		UINT8_C(0x01)

#define BMP3_INT_LEVEL_MSK			UINT8_C(0x02)
#define BMP3_INT_LEVEL_POS			UINT8_C(0x01)

#define BMP3_INT_LATCH_MSK			UINT8_C(0x04)
#define BMP3_INT_LATCH_POS			UINT8_C(0x02)

#define BMP3_INT_DRDY_EN_MSK			UINT8_C(0x40)
#define BMP3_INT_DRDY_EN_POS			UINT8_C(0x06)

#define BMP3_I2C_WDT_EN_MSK			UINT8_C(0x02)
#define BMP3_I2C_WDT_EN_POS			UINT8_C(0x01)

#define BMP3_I2C_WDT_SEL_MSK			UINT8_C(0x04)
#define BMP3_I2C_WDT_SEL_POS			UINT8_C(0x02)

#define BMP3_INT_STATUS_FWTM_MSK		UINT8_C(0x01)

#define BMP3_INT_STATUS_FFULL_MSK		UINT8_C(0x02)
#define BMP3_INT_STATUS_FFULL_POS		UINT8_C(0x01)

#define BMP3_INT_STATUS_DRDY_MSK		UINT8_C(0x08)
#define BMP3_INT_STATUS_DRDY_POS		UINT8_C(0x03)

#define	BMP3_SET_LOW_BYTE			UINT16_C(0x00FF)
#define	BMP3_SET_HIGH_BYTE			UINT16_C(0xFF00)

#define BMP3_CALIB_DATA_LEN			UINT8_C(21)
#define BMP3_P_AND_T_HEADER_DATA_LEN		UINT8_C(7)
#define BMP3_P_OR_T_HEADER_DATA_LEN		UINT8_C(4)
#define BMP3_P_T_DATA_LEN			UINT8_C(6)
#define BMP3_GEN_SETT_LEN			UINT8_C(7)
#define BMP3_P_DATA_LEN				UINT8_C(3)
#define BMP3_T_DATA_LEN				UINT8_C(3)
#define BMP3_SENSOR_TIME_LEN			UINT8_C(3)
#define BMP3_FIFO_MAX_FRAMES			UINT8_C(73)

#define BMP3_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)


#define BMP3_SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))

#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))

#define BMP3_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

#define BMP3_GET_LSB(var)	(uint8_t)(var & BMP3_SET_LOW_BYTE)
#define BMP3_GET_MSB(var)	(uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)

int bmp380_probe(struct device *dev,
			struct regmap *regmap,
			unsigned int chip,
			const char *name,
			int irq);
int bmp380_remove(struct device *dev);

/* Regmap configurations */
extern const struct regmap_config bmp380_regmap_config;

#endif  /* BMP380_H_ */
