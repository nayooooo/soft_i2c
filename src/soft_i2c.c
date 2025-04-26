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

#include "soft_i2c.h"

static void start(struct soft_i2c *si)
{
	si->ops->sda(1);
	si->ops->scl(1);
	si->ops->sda(0);
	si->ops->scl(0);
}

static void restart(struct soft_i2c *si)
{
	si->ops->sda(1);
	si->ops->scl(1);
	si->ops->sda(0);
	si->ops->scl(0);
}

static void stop(struct soft_i2c *si)
{
	si->ops->scl(0);
	si->ops->sda(0);
	si->ops->scl(1);
	si->ops->sda(1);
}

static uint8_t write_byte(struct soft_i2c *si, uint8_t byte, uint32_t x, void (*delay)(uint32_t x))
{
	for (int i = 0; i < 8; i++)
	{
		delay(x / 3);
		if (si->mode_of.xSB == SOFT_I2C_MSB)
		{
			si->ops->sda(!!((byte >> (7 - i)) & 0x1));
		}
		else
		{
			si->ops->sda(!!((byte >> i) & 0x1));
		}
		delay(x / 3);
		si->ops->scl(1);
		delay(x / 3);
		si->ops->scl(0);
	}
	
	si->ops->sda(1);  // prepare to read sda
	delay(x / 3);
	si->ops->scl(1);
	delay(x / 3);
	uint8_t ack = si->ops->sda(2);
	si->ops->scl(0);
	delay(x / 3);
	
	return (ack == SOFT_I2C_ACK) ? SOFT_I2C_ACK : SOFT_I2C_NACK;
}

static uint8_t read_byte(struct soft_i2c *si, uint8_t ack, uint32_t x, void (*delay)(uint32_t x))
{
	uint8_t data = 0x00;
	
	si->ops->sda(1);  // prepare to read sda
	for (int i = 0; i < 8; i++)
	{
		si->ops->scl(1);
		delay(x / 2);
		
		if (si->mode_of.xSB == SOFT_I2C_MSB)
		{
			data <<= 1;
			data |= si->ops->sda(2);
		}
		else
		{
			data >>= 1;
			data |= si->ops->sda(2) << 7;
		}
		
		si->ops->scl(0);
		delay(x / 2);
	}
	
	si->ops->sda(ack == SOFT_I2C_ACK ? SOFT_I2C_ACK : SOFT_I2C_NACK);
	delay(x / 3);
	si->ops->scl(1);
	delay(x / 3);
	si->ops->scl(0);
	delay(x / 3);
	
	return data;
}

static size_t soft_i2c_write(struct soft_i2c *si, uint32_t address, void *data, off_t offset, size_t size)
{
	size_t count = 0;
	uint8_t address_len = si->r_addr_size / 8 + (si->r_addr_size % 8 ? 1 : 0);
	uint8_t data_len = si->data_size / 8 + (si->data_size % 8 ? 1 : 0);
	
	// offset
	if (data != NULL)
		data = (uint8_t *)data + offset * (data_len == 1 ? 1 : data_len == 2 ? 2 : 4);
	
	// start
	start(si);
	
	// device address
	if (si->d_addr_size == SOFT_I2C_DEVICE_ADDRESS_SIZE_7)
	{
		if (SOFT_I2C_ACK != write_byte(si, (si->d_addr << 1) | SOFT_I2C_WRITE, si->x, si->delay))
		{
			goto _stop;
		}
	}
	else
	{
		if (SOFT_I2C_ACK != write_byte(si, 0xF0 | ((si->d_addr >> 8) << 1) | SOFT_I2C_WRITE, si->x, si->delay))
		{
			goto _stop;
		}
		if (SOFT_I2C_ACK != write_byte(si, si->d_addr & 0xFF, si->x, si->delay))
		{
			goto _stop;
		}
	}
	
	// register address
	for (int i = 0; i < address_len; i++)
	{
		if (si->mode_of.sr_endian == SOFT_I2C_LITTLE_ENDIAN)
		{
			if (SOFT_I2C_ACK != write_byte(si, (address >> (i * 8)) & 0xFF, si->x, si->delay))
			{
				goto _stop;
			}
		}
		else
		{
			if (SOFT_I2C_ACK != write_byte(si, (address >> ((address_len - 1 - i) * 8)) & 0xFF, si->x, si->delay))
			{
				goto _stop;
			}
		}
	}
	
	// data
	if (data == NULL) goto _stop;
	for (size_t n = 0; n < size; n++)
	{
		void *p = (uint8_t *)data + n * (data_len == 1 ? 1 : data_len == 2 ? 2 : 4);
		for (int i = 0; i < data_len; i++)
		{
			if (si->mode_of.m_endian == SOFT_I2C_LITTLE_ENDIAN)
			{
				if (si->mode_of.sd_endian == SOFT_I2C_LITTLE_ENDIAN)
				{
					if (SOFT_I2C_ACK != write_byte(si, ((uint8_t *)p)[i], si->x, si->delay))
					{
						goto _stop;
					}
				}
				else
				{
					if (SOFT_I2C_ACK != write_byte(si, ((uint8_t *)p)[data_len - 1 - i], si->x, si->delay))
					{
						goto _stop;
					}
				}
			}
			else
			{
				if (si->mode_of.sd_endian == SOFT_I2C_LITTLE_ENDIAN)
				{
					if (SOFT_I2C_ACK != write_byte(si, ((uint8_t *)p)[data_len - (data_len == 3 ? 0 : 1) - i], si->x, si->delay))
					{
						goto _stop;
					}
				}
				else
				{
					if (SOFT_I2C_ACK != write_byte(si, ((uint8_t *)p)[i + (data_len == 3 ? 1 : 0)], si->x, si->delay))
					{
						goto _stop;
					}
				}
			}
		}
		count ++;
	}
	
	// stop
_stop:
	stop(si);
	
	return count;
}

static size_t soft_i2c_read(struct soft_i2c *si, uint32_t address, void *data, off_t offset, size_t size)
{
	size_t count = 0;
	uint8_t address_len = si->r_addr_size / 8 + (si->r_addr_size % 8 ? 1 : 0);
	uint8_t data_len = si->data_size / 8 + (si->data_size % 8 ? 1 : 0);
	
	// offset
	if (data != NULL)
		data = (uint8_t *)data + offset * (data_len == 1 ? 1 : data_len == 2 ? 2 : 4);
	
	// start
	start(si);
	
	if (si->mode_of.has_dummy_write)
	{
	// device address
		if (si->d_addr_size == SOFT_I2C_DEVICE_ADDRESS_SIZE_7)
		{
			if (SOFT_I2C_ACK != write_byte(si, (si->d_addr << 1) | SOFT_I2C_WRITE, si->x, si->delay))
			{
				goto _stop;
			}
		}
		else
		{
			if (SOFT_I2C_ACK != write_byte(si, 0xF0 | ((si->d_addr >> 8) << 1) | SOFT_I2C_WRITE, si->x, si->delay))
			{
				goto _stop;
			}
			if (SOFT_I2C_ACK != write_byte(si, si->d_addr & 0xFF, si->x, si->delay))
			{
				goto _stop;
			}
		}
	
	// register address
		for (int i = 0; i < address_len; i++)
		{
			if (si->mode_of.sr_endian == SOFT_I2C_LITTLE_ENDIAN)
			{
				if (SOFT_I2C_ACK != write_byte(si, (address >> (i * 8)) & 0xFF, si->x, si->delay))
				{
					goto _stop;
				}
			}
			else
			{
				if (SOFT_I2C_ACK != write_byte(si, (address >> ((address_len - 1 - i) * 8)) & 0xFF, si->x, si->delay))
				{
					goto _stop;
				}
			}
		}
	
	// restart
		restart(si);
	}
	
	// device address
	if (si->d_addr_size == SOFT_I2C_DEVICE_ADDRESS_SIZE_7)
	{
		if (SOFT_I2C_ACK != write_byte(si, (si->d_addr << 1) | SOFT_I2C_READ, si->x, si->delay))
		{
			goto _stop;
		}
	}
	else
	{
		if (SOFT_I2C_ACK != write_byte(si, 0xF0 | ((si->d_addr >> 8) << 1) | SOFT_I2C_READ, si->x, si->delay))
		{
			goto _stop;
		}
		if (SOFT_I2C_ACK != write_byte(si, si->d_addr & 0xFF, si->x, si->delay))
		{
			goto _stop;
		}
	}
	
	// data
	if (data == NULL) goto _stop;
	for (size_t n = 0; n < size; n++)
	{
		void *p = (uint8_t *)data + n * (data_len == 1 ? 1 : data_len == 2 ? 2 : 4);
		for (int i = 0; i < data_len; i++)
		{
			uint8_t ack = (((n == size - 1) && (i == data_len - 1)) ? SOFT_I2C_NACK : SOFT_I2C_ACK);
			if (si->mode_of.sd_endian == SOFT_I2C_LITTLE_ENDIAN)
			{
				if (si->mode_of.m_endian == SOFT_I2C_LITTLE_ENDIAN)
				{
					((uint8_t *)p)[i] = read_byte(si, ack, si->x, si->delay);
				}
				else
				{
					((uint8_t *)p)[data_len - (data_len == 3 ? 0 : 1) - i] = read_byte(si, ack, si->x, si->delay);
				}
			}
			else
			{
				if (si->mode_of.m_endian == SOFT_I2C_LITTLE_ENDIAN)
				{
					((uint8_t *)p)[data_len - (data_len == 3 ? 0 : 1) - i] = read_byte(si, ack, si->x, si->delay);
				}
				else
				{
					((uint8_t *)p)[(data_len == 3 ? 1 : 0) + i] = read_byte(si, ack, si->x, si->delay);
				}
			}
		}
		count ++;
	}
	
	// stop
_stop:
	stop(si);
	
	return count;
}

int soft_i2c_init_ex(soft_i2c_t si, struct soft_i2c_pin_ops *ops,
					 uint32_t speed, uint32_t xSB,
					 uint32_t m_endian, uint32_t sr_endian, uint32_t sd_endian,
					 uint32_t has_dummy_write,
					 uint16_t d_addr, uint8_t d_addr_size,
					 uint8_t r_addr_size, uint8_t data_size,
					 void (*delay_ns)(uint32_t xns),
					 void (*delay_us)(uint32_t xus),
					 void (*delay_ms)(uint32_t xms))
{
	if (si == NULL || ops == NULL || ops->sda == NULL || ops->scl == NULL) return -1;
	if (!IS_LEGAL_SOFT_I2C_SPEED(speed)) return -2;
	if (!IS_SOFR_I2C_xSB(xSB)) return -3;
	if (!IS_SOFT_I2C_ENDIAN(m_endian)  ||\
		!IS_SOFT_I2C_ENDIAN(sr_endian) ||\
		!IS_SOFT_I2C_ENDIAN(sd_endian))
		return -4;
	if (has_dummy_write != 0 && has_dummy_write != 1) return -5;
	if (!IS_SOFT_I2C_DEVICE_ADDRESS_SIZE(d_addr_size)) return -6;
	if (!IS_SOFT_I2C_REGISTER_ADDRESS_SIZE(r_addr_size)) return -7;
	if (!IS_SOFT_I2C_DATA_SIZE(data_size)) return -8;
	if (delay_ns == NULL && delay_us == NULL && delay_ms == NULL) return -9;
	
	si->ops = ops;
	
	si->mode_of.speed = speed;
	si->mode_of.xSB = xSB;
	si->mode_of.m_endian = m_endian;
	si->mode_of.sr_endian = sr_endian;
	si->mode_of.sd_endian = sd_endian;
	si->mode_of.has_dummy_write = has_dummy_write;
	
	si->d_addr = d_addr & (d_addr_size == SOFT_I2C_DEVICE_ADDRESS_SIZE_7 ?\
						   0x7F : 0x3FF);
	si->d_addr_size = d_addr_size;
	si->r_addr_size = r_addr_size;
	si->data_size = data_size;
	
	si->mutex.count = 0;
	si->mutex.holder_handle = 0;
	
	si->write = soft_i2c_write;
	si->read  = soft_i2c_read;
	
	si->delay_ns = delay_ns;
	si->delay_us = delay_us;
	si->delay_ms = delay_ms;
	
	si->x = 1000000000U / si->mode_of.speed;
	si->delay = NULL;
	if (si->x < 1000)
	{
		if (si->delay_ns != NULL)
		{
			si->delay = si->delay_ns;
		}
	}
	else if (si->x < 1000000)
	{
		if (si->delay_us != NULL)
		{
			si->x /= 1000;
			si->delay = si->delay_us;
		}
	}
	else
	{
		if (si->delay_ms != NULL)
		{
			si->x /= 1000000;
			si->delay = si->delay_ms;
		}
	}
	if (si->delay == NULL)
	{
		if (si->delay_ns != NULL)
		{
			si->delay = si->delay_ns;
		}
		else if (si->delay_us != NULL)
		{
			si->x /= 1000;
			si->delay = si->delay_us;
		}
		else
		{
			si->x /= 1000000;
			si->delay = si->delay_ms;
		}
	}
	
	return 0;
}

int soft_i2c_init(soft_i2c_t si, struct soft_i2c_pin_ops *ops,
				  uint32_t speed, uint16_t d_addr,
				  void (*delay_ns)(uint32_t xns),
				  void (*delay_us)(uint32_t xus),
				  void (*delay_ms)(uint32_t xms))
{
	return soft_i2c_init_ex(si, ops, speed, SOFT_I2C_MSB,
							SOFT_I2C_LITTLE_ENDIAN, SOFT_I2C_LITTLE_ENDIAN, SOFT_I2C_LITTLE_ENDIAN,
							1,
							d_addr, SOFT_I2C_DEVICE_ADDRESS_SIZE_7,
							8, 8,
							delay_ns, delay_us, delay_ms);
}
