/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "board_config.h"

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdbool.h>

#include <nuttx/i2c/i2c_bitbang.h>
#include <nuttx/i2c/i2c_master.h>

#include <stm32_gpio.h>

#define GPIO_V12_I2C_SCL \
	(GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_25MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN2)

#define GPIO_V12_I2C_SDA \
	(GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_25MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN11)

struct vkfly_v12_i2c_bitbang_s {
	struct i2c_bitbang_lower_dev_s lower;
	struct i2c_master_s *master;
};

static void vkfly_v12_i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower);
static void vkfly_v12_i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower, bool high);
static void vkfly_v12_i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower, bool high);
static bool vkfly_v12_i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower);
static bool vkfly_v12_i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower);

static const struct i2c_bitbang_lower_ops_s g_vkfly_v12_i2c_bb_ops = {
	.initialize = vkfly_v12_i2c_bb_initialize,
	.set_scl = vkfly_v12_i2c_bb_set_scl,
	.set_sda = vkfly_v12_i2c_bb_set_sda,
	.get_scl = vkfly_v12_i2c_bb_get_scl,
	.get_sda = vkfly_v12_i2c_bb_get_sda,
};

static struct vkfly_v12_i2c_bitbang_s g_vkfly_v12_i2c_bitbang = {
	.lower = {
		.ops = &g_vkfly_v12_i2c_bb_ops,
		.priv = &g_vkfly_v12_i2c_bitbang,
	},
	.master = NULL,
};

static void vkfly_v12_i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower)
{
	(void)lower;

	px4_arch_configgpio(GPIO_V12_I2C_SCL);
	px4_arch_configgpio(GPIO_V12_I2C_SDA);
	px4_arch_gpiowrite(GPIO_V12_I2C_SCL, true);
	px4_arch_gpiowrite(GPIO_V12_I2C_SDA, true);

	if (!px4_arch_gpioread(GPIO_V12_I2C_SDA)) {
		for (int i = 0; i < 9; i++) {
			px4_arch_gpiowrite(GPIO_V12_I2C_SCL, true);
			up_udelay(10);
			px4_arch_gpiowrite(GPIO_V12_I2C_SCL, false);
			up_udelay(10);
		}

		px4_arch_gpiowrite(GPIO_V12_I2C_SCL, true);
		px4_arch_gpiowrite(GPIO_V12_I2C_SDA, true);
	}
}

static void vkfly_v12_i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower, bool high)
{
	(void)lower;
	px4_arch_gpiowrite(GPIO_V12_I2C_SCL, high);
}

static void vkfly_v12_i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower, bool high)
{
	(void)lower;
	px4_arch_gpiowrite(GPIO_V12_I2C_SDA, high);
}

static bool vkfly_v12_i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower)
{
	(void)lower;
	return px4_arch_gpioread(GPIO_V12_I2C_SCL);
}

static bool vkfly_v12_i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower)
{
	(void)lower;
	return px4_arch_gpioread(GPIO_V12_I2C_SDA);
}

struct i2c_master_s *vkfly_v12_i2cbus_initialize(int bus)
{
	if (bus == VKFLY_V12_I2C_BITBANG_BUS) {
		if (g_vkfly_v12_i2c_bitbang.master == NULL) {
			g_vkfly_v12_i2c_bitbang.master = i2c_bitbang_initialize(&g_vkfly_v12_i2c_bitbang.lower);
		}

		return g_vkfly_v12_i2c_bitbang.master;
	}

	return NULL;
}

int vkfly_v12_i2cbus_uninitialize(struct i2c_master_s *dev)
{
	if (dev == g_vkfly_v12_i2c_bitbang.master) {
		return 0;
	}

	return -ENODEV;
}
