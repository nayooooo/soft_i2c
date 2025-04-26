/*
 * Copyright (c) 2006-2021, nayooooo
 *
 * SPDX-License-Identifier: MIT
 *
 * @brief This class is used to implement an I2C host.
 * @not Only applicable for communication between hosts
 *		and slaves in big end or small end mode.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-24     nayooooo     the first version
 */

#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include <stdint.h>
#include <stddef.h>

#ifndef off_t
	typedef int off_t;
#endif  // !off_t

#define SOFT_I2C_ACK								(0)
#define SOFT_I2C_NACK								(1)

#define SOFT_I2C_DATA_BITS							(8)

/** @defgroup SOFT_I2C_RW_MODE
  * @{
  */
#define SOFT_I2C_WRITE								(0x0)
#define SOFT_I2C_READ								(0x1)
#define IS_SOFT_I2C_RW_MODE(RW)						(((RW) == SOFT_I2C_WRITE)					||\
													 ((RW) == SOFT_I2C_READ))
/**
  * @}
  */

/** @defgroup SOFT_I2C_SPEED
  * @{
  */
#define SOFT_I2C_SPEED_MIN							(100)
#define SOFT_I2C_SPEED_MAX							(400000)
#define IS_LEGAL_SOFT_I2C_SPEED(SPEED)				(((SPEED) >= SOFT_I2C_SPEED_MIN)			&&\
													 ((SPEED) <= SOFT_I2C_SPEED_MAX))
/**
  * @}
  */

/** @defgroup SOFT_I2C_xSB
  * @{
  */
#define SOFT_I2C_MSB								(0)
#define SOFT_I2C_LSB								(1)
#define IS_SOFR_I2C_xSB(xSB)						(((xSB) == SOFT_I2C_MSB) 					||\
													 ((xSB) == SOFT_I2C_LSB))
/**
  * @}
  */

/** @defgroup SOFT_I2C_ENDIAN
  * @{
  */
#define SOFT_I2C_LITTLE_ENDIAN						(0)
#define SOFT_I2C_BIG_ENDIAN							(1)
#define IS_SOFT_I2C_ENDIAN(ENDIAN)					(((ENDIAN) == SOFT_I2C_LITTLE_ENDIAN		||\
													 ((ENDIAN) == SOFT_I2C_BIG_ENDIAN)))
/**
  * @}
  */

/** @defgroup SOFT_I2C_DEVICE_ADDRESS_SIZE
  * @{
  */
#define SOFT_I2C_DEVICE_ADDRESS_SIZE_7				(7)
#define SOFT_I2C_DEVICE_ADDRESS_SIZE_10				(10)
#define IS_SOFT_I2C_DEVICE_ADDRESS_SIZE(ADDR)		(((ADDR) == SOFT_I2C_DEVICE_ADDRESS_SIZE_7	||\
													 ((ADDR) == SOFT_I2C_DEVICE_ADDRESS_SIZE_10)))
/**
  * @}
  */

/** @defgroup SOFT_I2C_REGISTER_ADDRESS_SIZE
  * @{
  */
#define SOFT_I2C_REGISTER_ADDRESS_SIZE_MIN			(1)
#define SOFT_I2C_REGISTER_ADDRESS_SIZE_MAX			(32)
#define IS_SOFT_I2C_REGISTER_ADDRESS_SIZE(SIZE)		(((SIZE) >= SOFT_I2C_REGISTER_ADDRESS_SIZE_MIN)	&&\
													 ((SIZE) <= SOFT_I2C_REGISTER_ADDRESS_SIZE_MAX))
/**
  * @}
  */

/** @defgroup  
  * @{
  */
#define SOFT_I2C_DATA_SIZE_MIN						(1)
#define SOFT_I2C_DATA_SIZE_MAX						(32)
#define IS_SOFT_I2C_DATA_SIZE(SIZE)					(((SIZE) >= SOFT_I2C_DATA_SIZE_MIN)			&&\
													 ((SIZE) <= SOFT_I2C_DATA_SIZE_MAX))
/**
  * @}
  */

// if the state not 0 or 1, then read the pin state
// if the return value not 0 or 1, means an error occurred while reading the pin state
struct soft_i2c_pin_ops
{
	uint8_t (*sda)(uint8_t state);
	uint8_t (*scl)(uint8_t state);
};

struct soft_i2c
{
	struct soft_i2c_pin_ops *ops;
	
	union
	{
		struct
		{
			uint32_t speed     : 19;		// @ref SOFT_I2C_SPEED
			uint32_t xSB   	   : 1;			// @ref SOFT_I2C_xSB
			uint32_t m_endian  : 1;			// master byte endianness, @ref SOFT_I2C_ENDIAN
			uint32_t sr_endian : 1;			// slave register byte endianness, @ref SOFT_I2C_ENDIAN
			uint32_t sd_endian : 1;			// slave data byte endianness, @ref SOFT_I2C_ENDIAN
			uint32_t has_dummy_write : 1;	// has dummy write before read
		} mode_of;
		uint32_t mode;
	};
	
	uint16_t d_addr;						// device address, low 7 bits or low 10 bits
	uint8_t d_addr_size;					// @ref SOFT_I2C_DEVICE_ADDRESS_SIZE
	uint8_t r_addr_size;					// @ref SOFT_I2C_REGISTER_ADDRESS_SIZE
	uint8_t data_size;						// @ref SOFT_I2C_DATA_SIZE
	
	struct
	{
		uint32_t count;
		uint32_t holder_handle;
	} mutex;
	
	size_t (*write)(struct soft_i2c *si, uint32_t address, void *data, off_t offset, size_t size);
	size_t (*read )(struct soft_i2c *si, uint32_t address, void *data, off_t offset, size_t size);
	
	void (*delay_ns)(uint32_t xns);
	void (*delay_us)(uint32_t xus);
	void (*delay_ms)(uint32_t xms);
	
	uint32_t x;
	void (*delay)(uint32_t x);
};
typedef struct soft_i2c *soft_i2c_t;

int soft_i2c_init_ex(soft_i2c_t si, struct soft_i2c_pin_ops *ops,
					 uint32_t speed, uint32_t xSB,
					 uint32_t m_endian, uint32_t sr_endian, uint32_t sd_endian,
					 uint32_t has_dummy_write,
					 uint16_t d_addr, uint8_t d_addr_size,
					 uint8_t r_addr_size, uint8_t data_size,
					 void (*delay_ns)(uint32_t xns),
					 void (*delay_us)(uint32_t xus),
					 void (*delay_ms)(uint32_t xms));

int soft_i2c_init(soft_i2c_t si, struct soft_i2c_pin_ops *ops,
				  uint32_t speed, uint16_t d_addr,
				  void (*delay_ns)(uint32_t xns),
				  void (*delay_us)(uint32_t xus),
				  void (*delay_ms)(uint32_t xms));

#endif  /* __SOFT_I2C_H__ */
