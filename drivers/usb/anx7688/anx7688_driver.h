/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.11

Filename : anx7688_driver.h

Project  : ANX7688

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android

Description:

Revision History:

******************************************************************************/
/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ANX7688_DRV_H
#define _ANX7688_DRV_H

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/async.h>

#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#define LOG_TAG "Anx7688"
//#define __func__  ""

unsigned char ReadReg(unsigned char RegAddr);
void WriteReg(unsigned char RegAddr, unsigned char RegVal);
int ReadBlockReg(u8 RegAddr, u8 len, u8 *dat);
int WriteBlockReg(u8 RegAddr, u8 len, const u8 *dat);
unchar is_cable_detected(void);
extern u8 sel_voltage_pdo_index;
extern u8 misc_status;
extern atomic_t anx7688_power_status;
void anx7688_power_standby(void);
void anx7688_hardware_poweron(void);
void anx7688_vbus_control(bool value);
#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data);
#endif
#endif
