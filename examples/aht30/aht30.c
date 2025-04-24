#include "aht30.h"

#define USE_TEST			(1)
#if USE_TEST
#	include "lora.h"
#	include <stdio.h>
#endif

static struct soft_i2c _si;

static uint8_t _sda(uint8_t state)
{
	if (state == 0 || state == 1)
		HAL_GPIO_WritePin(AHT30_SDA_GPIO_Port, AHT30_SDA_Pin, state == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(AHT30_SDA_GPIO_Port, AHT30_SDA_Pin);
}

static uint8_t _scl(uint8_t state)
{
	if (state == 0 || state == 1)
		HAL_GPIO_WritePin(AHT30_SCL_GPIO_Port, AHT30_SCL_Pin, state == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(AHT30_SCL_GPIO_Port, AHT30_SCL_Pin);
}

static struct soft_i2c_pin_ops _sipops = {
	_sda,
	_scl
};

int aht30_init(void)
{
	int ret = 0;
	
	HAL_Delay(5);
	
	ret = soft_i2c_init_ex(&_si, &_sipops, 100000, SOFT_I2C_MSB,
						   SOFT_I2C_LITTLE_ENDIAN, SOFT_I2C_LITTLE_ENDIAN, SOFT_I2C_LITTLE_ENDIAN,
						   0,
						   0x38, SOFT_I2C_DEVICE_ADDRESS_SIZE_7,
						   8, 8,
						   NULL,
						   HAL_Delay_us,
						   HAL_Delay);
	if (ret != 0) return ret;
	
#if USE_TEST
	// test begin
	uint8_t data[7] = { 0x33, 0x00 };
	_si.write(&_si, 0xAC, (void *)&data[0], 0, 2);
	HAL_Delay(150);
	_si.read(&_si, 0xAC, (void *)&data[0], 0, 7);
	char str[24] = "";
	for (int i = 0; i < 7; i++)
	{
		sprintf(&str[i * 3], "%02X ", data[i]);
		if (i == 6)
		{
			str[(i + 1) * 3 + 0] = '\r';
			str[(i + 1) * 3 + 1] = '\n';
			str[(i + 1) * 3 + 2] = '\0';
		}
	}
	lora_log("AHT30 ori", (const char *)str); HAL_Delay(40);
	uint32_t res = 0;
	crc8_aht30((const uint8_t *)&data[0], 6, &res);
	if (res != (uint32_t)data[6])
	{
		lora_log("AHT30", "CRC error!"); HAL_Delay(40);
		return -1;
	}
	float rh = (float)((data[1] << 12) | (data[2] << 4) | ((data[3] & 0xF0) >> 4)) /\
			   1048576 * 100;
	float t  = (float)(((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) /\
			   1048576 * 200 - 50;
	sprintf(str, "RH: %.2f%%, t: %.2fC", rh, t);
	lora_log("AHT30", (const char *)str); HAL_Delay(40);
	// test end
#endif
	
	return 0;
}
