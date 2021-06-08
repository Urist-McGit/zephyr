/*
 * Copyright (c) 2021 Christoph Steiger
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <drivers/can.h>
#include <soc.h>

#include "can_samv71.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(can_driver, CONFIG_CAN_LOG_LEVEL);


#define DT_DRV_COMPAT atmel_samv71_mcan

#define PCK5_ID 5
#define UPLL_CLK_PRESCALER 11

#define CAN_CLK_SWITCH_TIMEOUT_MS 100

static int pmc_switch_pck_to_upllck(uint32_t id, uint32_t pres)
{
	uint32_t start_time;
	k_timeout_t timeout;

	PMC->PMC_PCK[id] = PMC_PCK_CSS_UPLL_CLK | pres;
	start_time = k_cycle_get_32();
	timeout = K_MSEC(CAN_CLK_SWITCH_TIMEOUT_MS);
	while (!(PMC->PMC_SR & (PMC_SR_PCKRDY0_Msk << id))) {
		if (k_cycle_get_32() - start_time > timeout.ticks) {
			return CAN_TIMEOUT;
		}
	}

	return 0;
}

void can_samv71_register_state_change_isr(const struct device *dev,
					   can_state_change_isr_t isr)
{
	struct can_samv71_data *data = DEV_DATA(dev);

	data->mcan_data.state_change_isr = isr;
}

static int can_samv71_init(const struct device *dev)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;
	int ret;

	/* Connect pins to the peripheral */
	soc_gpio_list_configure(cfg->pin_list, cfg->pin_list_size);

	/* Start the USB PLL */
	PMC->CKGR_UCKR |= CKGR_UCKR_UPLLEN;

	/* Wait for it to be ready */
	while (!(PMC->PMC_SR & PMC_SR_LOCKU)) {
		k_yield();
	}

	/* Select UPLL as source clock, with prescaler of either 5, 11 or 23 */
	PMC->PMC_SCDR |= PMC_SCDR_PCK5_Msk;
	ret = pmc_switch_pck_to_upllck(
			PCK5_ID, PMC_PCK_PRES(UPLL_CLK_PRESCALER));
	if (ret) {
		LOG_ERR("Failed to switch clock source");
		return -EIO;
	}
	PMC->PMC_SCER |= PMC_SCER_PCK5_Msk;
	soc_pmc_peripheral_enable(cfg->peripheral_id);

	ret = can_mcan_init(dev, mcan_cfg, msg_ram, mcan_data);
	if (ret) {
		return ret;
	}

	cfg->irq_config();

	return ret;
}

static void can_samv71_line_0_isr(const struct device *dev)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	can_mcan_line_0_isr(mcan_cfg, msg_ram, mcan_data);
}

static void can_samv71_line_1_isr(const struct device *dev)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	can_mcan_line_1_isr(mcan_cfg, msg_ram, mcan_data);
}

int can_samv71_set_mode(const struct device *dev, enum can_mode mode)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_set_mode(mcan_cfg, mode);
}

int can_samv71_set_timing(const struct device *dev,
		const struct can_timing *timing,
		const struct can_timing* timing_data)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_set_timing(mcan_cfg, timing, timing_data);
}

int can_samv71_send(const struct device *dev, const struct zcan_frame *frame,
		k_timeout_t timeout, can_tx_callback_t callback,
		void *callback_arg)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	return can_mcan_send(mcan_cfg, mcan_data, msg_ram, frame, timeout,
			callback, callback_arg);
}

int can_samv71_attach_isr(const struct device *dev, can_rx_callback_t isr,
		void *cb_arg, const struct zcan_filter *filter)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	return can_mcan_attach_isr(mcan_data, msg_ram, isr, cb_arg, filter);
}

void can_samv71_detach(const struct device *dev, int filter_nr)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	return can_mcan_detach(mcan_data, msg_ram, filter_nr);
}

enum can_state can_samv71_get_state(const struct device * dev,
		struct can_bus_err_cnt *err_cnt)
{
	const struct can_samv71_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_get_state(mcan_cfg, err_cnt);
}

static int can_samv71_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);
        *rate = SOC_ATMEL_SAM_UPLL_FREQ_HZ / (UPLL_CLK_PRESCALER + 1);

	return 0;
}

static const struct can_driver_api can_api_funcs = {
	.set_mode = can_samv71_set_mode,
	.set_timing = can_samv71_set_timing,
	.send = can_samv71_send,
	.attach_isr = can_samv71_attach_isr,
	.detach = can_samv71_detach,
	.get_state = can_samv71_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_mcan_recover,
#endif
	.get_core_clock = can_samv71_get_core_clock,
	.register_state_change_isr = can_samv71_register_state_change_isr,
	.timing_min = {
		.sjw = 0x7f,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01 },
#ifdef CONFIG_SOC_PART_NUMBER_SAMV71Q21
	.timing_max= {
		.sjw = 0x0f,
		.prop_seg = 0x00,
		.phase_seg1 = 0x40,
		.phase_seg2 = 0x10,
		.prescaler = 0x400 },
#else
	.timing_max= {
		.sjw = 0x7f,
		.prop_seg = 0x00,
		.phase_seg1 = 0x100,
		.phase_seg2 = 0x80,
		.prescaler = 0x200 },
#endif /* CONFIG_SOC_PART_NUMBER_SAMV71Q21 */
#ifdef CONFIG_CAN_FD_MODE
	.timing_min_data = {
		.sjw = 0x01,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01 },
#ifdef CONFIG_SOC_PART_NUMBER_SAMV71Q21
	.timing_max_data = {
		.sjw = 0x4,
		.prop_seg = 0x00,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x8,
		.prescaler = 0x20 }
#else
	.timing_max_data = {
		.sjw = 0x10,
		.prop_seg = 0x00,
		.phase_seg1 = 0x20,
		.phase_seg2 = 0x10,
		.prescaler = 0x20 }
#endif /* CONFIG_SOC_PART_NUMBER_SAMV71Q21 */
#endif /* CONFIG_CAN_FD_MODE */
};

#ifdef CONFIG_CAN_FD_MODE
#define SAMV71_MCAN_FD_CONFIG(inst)                                            \
	.bus_speed_data = DT_INST_PROP(inst, bus_speed_data),                  \
	.sjw_data = DT_INST_PROP(inst, sjw_data),                              \
	.sample_point_data = DT_INST_PROP_OR(inst, sample_point_data, 0),      \
	.prop_ts1_data = DT_INST_PROP_OR(inst, prop_seg_data, 0) +             \
			 DT_INST_PROP_OR(inst, phase_seg1_data, 0),            \
	.ts2_data = DT_INST_PROP_OR(inst, phase_seg2_data, 0),
#else /* CONFIG_CAN_FD_MODE */
#define SAMV71_MCAN_FD_CONFIG(inst)
#endif /* CONFIG_CAN_FD_MODE */

#define SAMV71_MCAN_INIT(inst)                                                 \
	static void can_samv71_irq_config_##inst()                             \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, line_0, irq),            \
			    DT_INST_IRQ_BY_NAME(inst, line_0, priority),       \
			    can_samv71_line_0_isr,                             \
			    DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, line_0, irq));            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, line_1, irq),            \
			    DT_INST_IRQ_BY_NAME(inst, line_1, priority),       \
			    can_samv71_line_1_isr,                             \
			    DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, line_1, irq));            \
	}                                                                      \
                                                                               \
	struct can_mcan_msg_sram __attribute__((__section__(".can_msg_sram"))) \
		can_samv71_msg_sram_##inst __aligned(4);                       \
	static const struct can_samv71_config can_samv71_cfg_##inst = {        \
		.msg_sram = &can_samv71_msg_sram_##inst,                       \
		.mcan_cfg = { .can = (struct can_mcan_reg *)                   \
				      DT_INST_REG_ADDR_BY_NAME(inst, m_can),   \
				.bus_speed = DT_INST_PROP(inst, bus_speed),    \
				.sjw = DT_INST_PROP(inst, sjw),                \
				.sample_point =                                \
				      DT_INST_PROP_OR(inst, sample_point, 0),  \
				.prop_ts1 =                                    \
				      DT_INST_PROP_OR(inst, prop_seg, 0) +     \
				      DT_INST_PROP_OR(inst, phase_seg1, 0),    \
				.ts2 = DT_INST_PROP_OR(inst, phase_seg2, 0),   \
				SAMV71_MCAN_FD_CONFIG(inst) },                 \
		.irq_config = can_samv71_irq_config_##inst,                    \
		.peripheral_id = DT_INST_PROP(inst, peripheral_id),            \
		.pin_list_size = ATMEL_SAM_DT_NUM_PINS(inst),                  \
		.pin_list = ATMEL_SAM_DT_PINS(inst)                            \
	};                                                                     \
	static struct can_samv71_data can_samv71_data_##inst;                  \
	DEVICE_DT_INST_DEFINE(inst, &can_samv71_init, NULL,                    \
			    &can_samv71_data_##inst, &can_samv71_cfg_##inst,   \
			    POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,             \
			    &can_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(SAMV71_MCAN_INIT)
