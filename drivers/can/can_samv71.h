/*
 * Copyright (c) 2021 Christoph Steiger
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_CAN_SAMV71_H_
#define ZEPHYR_DRIVERS_CAN_SAMV71_H_

#include "can_mcan.h"

#define DEV_DATA(dev) ((struct can_samv71_data *)(dev)->data)
#define DEV_CFG(dev) ((const struct can_samv71_config *)(dev)->config)

struct can_samv71_config {
	struct can_mcan_msg_sram *msg_sram;
	struct can_mcan_config mcan_cfg;
	void (*irq_config)();
	uint32_t peripheral_id;
	uint32_t pin_list_size;
	struct soc_gpio_pin pin_list[];
};

struct can_samv71_data {
	struct can_mcan_data mcan_data;
};

#endif /* ZEPHYR_DRIVERS_CAN_SAMV71_H_ */
