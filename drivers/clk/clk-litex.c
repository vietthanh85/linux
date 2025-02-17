// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Antmicro Ltd. <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#include "clk-litex.h"
#include <linux/litex.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

struct litex_drp_reg {
	u32 offset;
	u32 size;
};

struct litex_drp_reg drp[] = {
	{DRP_OF_RESET,  DRP_SIZE_RESET},
	{DRP_OF_READ,   DRP_SIZE_READ},
	{DRP_OF_WRITE,  DRP_SIZE_WRITE},
	{DRP_OF_DRDY,   DRP_SIZE_DRDY},
	{DRP_OF_ADR,    DRP_SIZE_ADR},
	{DRP_OF_DAT_W,  DRP_SIZE_DAT_W},
	{DRP_OF_DAT_R,  DRP_SIZE_DAT_R},
	{DRP_OF_LOCKED, DRP_SIZE_LOCKED},
};

struct litex_clk_device {
	void __iomem *base;
	int is_clkout;
	struct clk_hw clk_hw;
	struct litex_clk_clkout *clkouts;
	u32 nclkout;
	u32 lock_timeout;
	u32 drdy_timeout;
	u32 sys_clk_freq;
};

struct litex_clk_clkout_addr {
	u8 reg1;
	u8 reg2;
};

struct litex_clk_regs_addr {
	struct litex_clk_clkout_addr clkout[CLKOUT_MAX];
};

struct litex_clk_clkout {
	void __iomem *base;
	int is_clkout;
	struct clk_hw clk_hw;
	u32 id;
	u32 default_freq;
	u32 default_phase;
	u32 default_duty_num;
	u32 default_duty_den;
	u32 lock_timeout;
	u32 drdy_timeout;
	u32 sys_clk_freq;
};

struct litex_clk_regs_addr litex_clk_regs_addr_init(void)
{
	struct litex_clk_regs_addr m;
	u32 i, addr;

	addr = CLKOUT0_REG1;
	for (i = 0; i <= CLKOUT_MAX; i++) {
		if (i == 5) {
		/*
		 *special case because CLKOUT5 have its reg addresses
		 *placed lower than other CLKOUTs
		 */
			m.clkout[5].reg1 = CLKOUT5_REG1;
			m.clkout[5].reg2 = CLKOUT5_REG2;
		} else {
			m.clkout[i].reg1 = addr;
			addr++;
			m.clkout[i].reg2 = addr;
			addr++;
		}
	}
	return m;
}

/*
 * This code is taken from:
 * https://github.com/torvalds/linux/blob/master/drivers/clk/clk-axi-clkgen.c
 *
 *	Copyright 2012-2013 Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * FIXME: make this code common
 */

/* MMCM loop filter lookup table */
static u32 litex_clk_lookup_filter(u32 glob_mul)
{
	switch (glob_mul) {
	case 0:
		return 0x01001990;
	case 1:
		return 0x01001190;
	case 2:
		return 0x01009890;
	case 3:
		return 0x01001890;
	case 4:
		return 0x01008890;
	case 5 ... 8:
		return 0x01009090;
	case 9 ... 11:
		return 0x01000890;
	case 12:
		return 0x08009090;
	case 13 ... 22:
		return 0x01001090;
	case 23 ... 36:
		return 0x01008090;
	case 37 ... 46:
		return 0x08001090;
	default:
		return 0x08008090;
	}
}

/* MMCM lock detection lookup table */
static const u32 litex_clk_lock_table[] = {
	0x060603e8, 0x060603e8, 0x080803e8, 0x0b0b03e8,
	0x0e0e03e8, 0x111103e8, 0x131303e8, 0x161603e8,
	0x191903e8, 0x1c1c03e8, 0x1f1f0384, 0x1f1f0339,
	0x1f1f02ee, 0x1f1f02bc, 0x1f1f028a, 0x1f1f0271,
	0x1f1f023f, 0x1f1f0226, 0x1f1f020d, 0x1f1f01f4,
	0x1f1f01db, 0x1f1f01c2, 0x1f1f01a9, 0x1f1f0190,
	0x1f1f0190, 0x1f1f0177, 0x1f1f015e, 0x1f1f015e,
	0x1f1f0145, 0x1f1f0145, 0x1f1f012c, 0x1f1f012c,
	0x1f1f012c, 0x1f1f0113, 0x1f1f0113, 0x1f1f0113,
};

/* Helper function for lock lookup table */
static u32 litex_clk_lookup_lock(u32 glob_mul)
{
	if (glob_mul < ARRAY_SIZE(litex_clk_lock_table))
		return litex_clk_lock_table[glob_mul];
	return 0x1f1f00fa;
}
/* End of copied code */

static inline struct litex_clk_device *clk_hw_to_litex_clk_device
						  (struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct litex_clk_device, clk_hw);
}

static inline struct litex_clk_clkout *clk_hw_to_litex_clk_clkout
						  (struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct litex_clk_clkout, clk_hw);
}

/* Clock control driver functions */
static inline void litex_clk_set_reg(struct clk_hw *clk_hw, u32 reg, u32 val)
{
	struct litex_clk_clkout *l_clk = clk_hw_to_litex_clk_clkout(clk_hw);

	_litex_set_reg(l_clk->base + drp[reg].offset, drp[reg].size, val);
}

static inline u32 litex_clk_get_reg(struct clk_hw *clk_hw, u32 reg)
{
	struct litex_clk_clkout *l_clk = clk_hw_to_litex_clk_clkout(clk_hw);

	return _litex_get_reg(l_clk->base + drp[reg].offset, drp[reg].size);
}

static inline void litex_clk_assert_reg(struct clk_hw *clk_hw, u32 reg)
{
	int assert = (1 << (drp[reg].size * BITS_PER_BYTE)) - 1;

	litex_clk_set_reg(clk_hw, reg, assert);
}

static inline void litex_clk_deassert_reg(struct clk_hw *clk_hw, u32 reg)
{
	litex_clk_set_reg(clk_hw, reg, ZERO_REG);
}

static int litex_clk_wait_lock(struct clk_hw *clk_hw)
{
	struct litex_clk_clkout *l_clk;
	u32 timeout;

	l_clk = clk_hw_to_litex_clk_clkout(clk_hw);

	/* Check if clk_hw is global or clkout specific */
	if (l_clk->is_clkout) {
		timeout = l_clk->lock_timeout;
	} else {
		/* clk hw is global */
		struct litex_clk_device *l_dev;

		l_dev = clk_hw_to_litex_clk_device(clk_hw);
		timeout = l_dev->lock_timeout;
	}

	/*Waiting for LOCK signal to assert in MMCM*/
	while (!litex_clk_get_reg(clk_hw, DRP_LOCKED) && timeout) {
		timeout--;
		usleep_range(900, 1000);
	}
	if (timeout == 0) {
		pr_err("CLKOUT%d: %s timeout", l_clk->id, __func__);
		return -ETIME;
	}
	return 0;
}

static int litex_clk_wait_drdy(struct clk_hw *clk_hw)
{
	struct litex_clk_clkout *l_clk;
	u32 timeout;

	l_clk = clk_hw_to_litex_clk_clkout(clk_hw);

	/* Check if clk_hw is global or clkout specific */
	if (l_clk->is_clkout) {
		timeout = l_clk->drdy_timeout;
	} else {
		/* clk hw is global */
		struct litex_clk_device *l_dev;

		l_dev = clk_hw_to_litex_clk_device(clk_hw);
		timeout = l_dev->drdy_timeout;
	}

	/*Waiting for DRDY signal to assert in MMCM*/
	while (!litex_clk_get_reg(clk_hw, DRP_DRDY) && timeout) {
		timeout--;
		usleep_range(900, 1000);
	}
	if (timeout == 0) {
		pr_err("CLKOUT%d: %s timeout", l_clk->id, __func__);
		return -ETIME;
	}
	return 0;
}

/* Read value written in given internal MMCM register*/
static int litex_clk_get_DO(struct clk_hw *clk_hw, u8 clk_reg_addr, u16 *res)
{
	int ret;

	litex_clk_set_reg(clk_hw, DRP_ADR, clk_reg_addr);
	litex_clk_assert_reg(clk_hw, DRP_READ);

	litex_clk_deassert_reg(clk_hw, DRP_READ);
	ret = litex_clk_wait_drdy(clk_hw);
	if (ret != 0)
		return ret;

	*res = litex_clk_get_reg(clk_hw, DRP_DAT_R);

	return 0;
}

/* Return global divider and multiplier values */
static int litex_clk_get_global_divider(struct clk_hw *hw, u32 *divider,
							   u32 *multiplier)
{
	int ret;
	u16 divreg, mult;
	u8 low_time, high_time;

	ret = litex_clk_get_DO(hw, CLKFBOUT_REG1, &mult);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, DIV_REG, &divreg);
	if (ret != 0)
		return ret;

	low_time = mult & HL_TIME_MASK;
	high_time = (mult >> HIGH_TIME_POS) & HL_TIME_MASK;
	*multiplier = low_time + high_time;

	low_time = divreg & HL_TIME_MASK;
	high_time = (divreg >> HIGH_TIME_POS) & HL_TIME_MASK;
	*divider = low_time + high_time;

	return 0;
}

/* Calculate frequency after global divider and multiplier */
static u32 litex_clk_get_real_global_frequency(struct clk_hw *hw)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	u32 g_divider, g_multiplier;

	litex_clk_get_global_divider(hw, &g_divider, &g_multiplier);
	return l_clkout->sys_clk_freq * g_multiplier / g_divider;
}

/* Calculate frequency after global divider and multiplier without clk_hw */
static u32 litex_clk_get_expctd_global_frequency(u32 sys_clk_freq)
{
	u32 g_divider, g_multiplier;

	g_divider = (GLOB_DIV_VAL & HL_TIME_MASK) +
		   ((GLOB_DIV_VAL >> HIGH_TIME_POS) & HL_TIME_MASK);

	g_multiplier = (GLOB_MUL_VAL & HL_TIME_MASK) +
		      ((GLOB_MUL_VAL >> HIGH_TIME_POS) & HL_TIME_MASK);

	return sys_clk_freq * g_multiplier / g_divider;
}
/* Return dividers of given CLKOUT */
static int litex_clk_get_clkout_divider(struct clk_hw *hw, u32 *divider,
							   u32 *fract_cnt)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u16 div, frac;
	u8 clkout_nr = l_clkout->id;
	u8 low_time, high_time;

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1, &div);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2, &frac);
	if (ret != 0)
		return ret;

	low_time = div & HL_TIME_MASK;
	high_time = (div >> HIGH_TIME_POS) & HL_TIME_MASK;
	*divider = low_time + high_time;
	*fract_cnt = (frac >> FRAC_POS) & FRAC_MASK;

	return 0;
}

/* Debug functions */
#ifdef DEBUG
static void litex_clk_print_general_regs(struct clk_hw *clk_hw)
{
	u16 power_reg, div_reg, clkfbout_reg, lock_reg1,
	lock_reg2, lock_reg3, filt_reg1, filt_reg2;

	litex_clk_get_DO(clk_hw, POWER_REG, &power_reg);
	litex_clk_get_DO(clk_hw, DIV_REG, &div_reg);
	litex_clk_get_DO(clk_hw, CLKFBOUT_REG1, &clkfbout_reg);
	litex_clk_get_DO(clk_hw, LOCK_REG1, &lock_reg1);
	litex_clk_get_DO(clk_hw, LOCK_REG2, &lock_reg2);
	litex_clk_get_DO(clk_hw, LOCK_REG3, &lock_reg3);
	litex_clk_get_DO(clk_hw, FILT_REG1, &filt_reg1);
	litex_clk_get_DO(clk_hw, FILT_REG2, &filt_reg2);

	pr_debug("POWER REG:  0x%x\n", power_reg);
	pr_debug("DIV REG:    0x%x\n", div_reg);
	pr_debug("MUL REG:    0x%x\n", clkfbout_reg);
	pr_debug("LOCK_REG1:  0x%x\n", lock_reg1);
	pr_debug("LOCK_REG2:  0x%x\n", lock_reg2);
	pr_debug("LOCK_REG3:  0x%x\n", lock_reg3);
	pr_debug("FILT_REG1:  0x%x\n", filt_reg1);
	pr_debug("FILT_REG2:  0x%x\n", filt_reg2);
}

static void litex_clk_print_clkout_regs(struct clk_hw *clk_hw, u8 clkout,
						      u8 reg1, u8 reg2)
{
	u16 clkout_reg1, clkout_reg2;

	litex_clk_get_DO(clk_hw, reg1, &clkout_reg1);
	pr_debug("CLKOUT%d REG1: 0x%x\n", clkout, clkout_reg1);

	litex_clk_get_DO(clk_hw, reg1, &clkout_reg2);
	pr_debug("CLKOUT%d REG2: 0x%x\n", clkout, clkout_reg2);
}

static void litex_clk_print_all_regs(struct clk_hw *clk_hw)
{
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	u32 i;

	pr_debug("General regs:\n");
	litex_clk_print_general_regs(clk_hw);
	for (i = 0; i < CLKOUT_MAX; i++)
		litex_clk_print_clkout_regs(clk_hw, i, drp_addr.clkout[i].reg1,
						       drp_addr.clkout[i].reg2);
}
#endif /* #ifdef DEBUG */

/* Return raw value ready to be written into DRP */
static inline u16 litex_clk_calc_DI(u16 DO_val, u16 mask, u16 bitset)
{
	u16 DI_val;

	DI_val = DO_val & mask;
	DI_val |= bitset;

	return DI_val;
}

/* Sets calculated DI value into DI DRP register */
static int litex_clk_set_DI(struct clk_hw *clk_hw, u16 DI_val)
{
	int ret;

	litex_clk_set_reg(clk_hw, DRP_DAT_W, DI_val);
	litex_clk_assert_reg(clk_hw, DRP_WRITE);
	litex_clk_deassert_reg(clk_hw, DRP_WRITE);
	ret = litex_clk_wait_drdy(clk_hw);
	if (ret != 0)
		return ret;
	return 0;
}

/*
 * Change register value as specified in arguments
 *
 * clk_hw:		hardware specific clock structure for selecting CLKOUT
 * mask:		preserve or zero MMCM register bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset:		set those bits in MMCM register which are 1 in bitset
 * clk_reg_addr:	internal MMCM address of control register
 *
 */
static int litex_clk_change_value(struct clk_hw *clk_hw, u16 mask, u16 bitset,
							  u8 clk_reg_addr)
{
	u16 DO_val, DI_val;
	int ret;

	litex_clk_assert_reg(clk_hw, DRP_RESET);

	ret = litex_clk_get_DO(clk_hw, clk_reg_addr, &DO_val);
	if (ret != 0)
		return ret;
	DI_val = litex_clk_calc_DI(DO_val, mask, bitset);
	ret = litex_clk_set_DI(clk_hw, DI_val);
	if (ret != 0)
		return ret;
#ifdef DEBUG
	DI_val = litex_clk_get_reg(clk_hw, DRP_DAT_W);
#endif
	litex_clk_deassert_reg(clk_hw, DRP_DAT_W);
	litex_clk_deassert_reg(clk_hw, DRP_RESET);
	ret = litex_clk_wait_lock(clk_hw);
	if (ret != 0)
		return ret;

	pr_debug("set 0x%x under addr: 0x%x", DI_val, clk_reg_addr);
	return 0;
}

/*
 * Set register values for given CLKOUT
 *
 * clk_hw:		hardware specific clock structure for selecting CLKOUT
 * clkout_nr:		clock output number
 * mask_regX:		preserve or zero MMCM register X bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset_regX:		set those bits in MMCM register X  which are 1 in bitset
 *
 */
static int litex_clk_set_clock(struct clk_hw *clk_hw, u8 clkout_nr,
					u16 mask_reg1, u16 bitset_reg1,
					u16 mask_reg2, u16 bitset_reg2)
{
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;

	if (!(mask_reg2 == KEEP_REG_16 && bitset_reg2 == ZERO_REG)) {
		ret = litex_clk_change_value(clk_hw, mask_reg2, bitset_reg2,
					    drp_addr.clkout[clkout_nr].reg2);
		if (ret != 0)
			return ret;
	}
	if (!(mask_reg1 == KEEP_REG_16 && bitset_reg1 == ZERO_REG)) {
		ret = litex_clk_change_value(clk_hw, mask_reg1, bitset_reg1,
					drp_addr.clkout[clkout_nr].reg1);
		if (ret != 0)
			return ret;
	}
	return 0;
}

/*
 * Set global divider for all CLKOUTs
 *
 * clk_hw:		hardware specific clock structure
 * mask:		preserve or zero MMCM register bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset:		set those bits in MMCM register which are 1 in bitset
 *
 */
static int litex_clk_set_divreg(struct clk_hw *clk_hw, u16 mask, u16 bitset)
{
	int ret;

	ret = litex_clk_change_value(clk_hw, mask, bitset, DIV_REG);
	if (ret != 0)
		return ret;
	pr_debug("Global divider set to %d", (bitset % (1 << HIGH_TIME_POS)) +
			  ((bitset >> HIGH_TIME_POS) % (1 << HIGH_TIME_POS)));
	return 0;
}

/*
 * Set global multiplier for all CLKOUTs
 *
 * clk_hw:		hardware specific clock structure
 * mask:		preserve or zero MMCM register bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset:		set those bits in MMCM register which are 1 in bitset
 *
 */
static int litex_clk_set_mulreg(struct clk_hw *clk_hw, u16 mask, u16 bitset)
{
	int ret;

	ret = litex_clk_change_value(clk_hw, mask, bitset, CLKFBOUT_REG1);
	if (ret != 0)
		return ret;
	pr_debug("Global multiplier set to %d",
		(bitset % (1 << HIGH_TIME_POS)) + ((bitset >> HIGH_TIME_POS) %
							(1 << HIGH_TIME_POS)));
	return 0;
}

static int litex_clk_set_filt(struct clk_hw *hw)
{
	u32 filt, mul;
	int ret;

	mul = GLOB_MUL_VAL && HL_TIME_MASK;
	mul += (GLOB_MUL_VAL >> HIGH_TIME_POS) && HL_TIME_MASK;

	filt = litex_clk_lookup_filter(mul - 1);

	ret = litex_clk_change_value(hw, FILT_MASK, filt >> 16, FILT_REG1);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, FILT_MASK, filt, FILT_REG2);
	if (ret != 0)
		return ret;
	return 0;
}

static int litex_clk_set_lock(struct clk_hw *hw)
{
	u32 lock, mul;
	int ret;

	mul = GLOB_MUL_VAL && HL_TIME_MASK;
	mul += (GLOB_MUL_VAL >> HIGH_TIME_POS) && HL_TIME_MASK;

	lock = litex_clk_lookup_lock(mul - 1);

	ret = litex_clk_change_value(hw, LOCK1_MASK, lock & 0x3ff, LOCK_REG1);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, LOCK23_MASK,
				(((lock >> 16) & 0x1f) << 10) | 0x1, LOCK_REG2);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, LOCK23_MASK,
			      (((lock >> 24) & 0x1f) << 10) | 0x3e9, LOCK_REG3);
	if (ret != 0)
		return ret;
	return 0;
}

/*
 * Load default global divider and multiplier values
 *
 */
static int litex_clk_set_dm(struct clk_hw *clk_hw)
{
	int ret;

	ret = litex_clk_set_divreg(clk_hw, KEEP_IN_DIV, GLOB_DIV_VAL);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_mulreg(clk_hw, KEEP_IN_MUL, GLOB_MUL_VAL);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_filt(clk_hw);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_lock(clk_hw);
	if (ret != 0)
		return ret;
	return 0;
}

/* Round scaled value*/
static inline u32 litex_round(u32 val, u32 mod)
{
	if (val % mod > mod / 2)
		return val / mod + 1;
	else
		return val / mod;
}

/*
 *	Duty Cycle
 */

/*
 * Calculate necessary values for setting duty cycle in fractional mode
 *
 * divider:		frequency divider value for calculating the phase
 * high_duty:		desired duty cycle in percent
 * fract_cnt:		fractional frequency divider value, if not for
 *			CLKOUT0 - set 0
 * pointers:		function return values, used for setting desired duty
 *
 * This function is based on Xilinx Unified Simulation Library Component
 * Advanced Mixed Mode Clock Manager (MMCM) taken from Xilinx Vivado
 * found in: <Vivado-dir>/data/verilog/src/unisims/MMCME2_ADV.v
 *
 */
static int litex_clk_calc_duty_fract(int divider, int high_duty, int fract_cnt,
					u8 *edge, u8 *high_time, u8 *low_time,
					u8 *frac_wf_r, u8 *frac_wf_f)
{
	int even_part_high, even_part_low, odd, odd_and_frac;

			/* divider / 2 */
	even_part_high = divider >> 1;
	even_part_low = even_part_high;
	odd = divider - even_part_high - even_part_low;
	odd_and_frac = (8 * odd) + fract_cnt;

	*low_time = even_part_high;
	if (odd_and_frac <= ODD_AND_FRAC)
		(*low_time)--;

	*high_time = even_part_low;
	if (odd_and_frac <= EVEN_AND_FRAC)
		(*high_time)--;

	*edge = divider % 2;

	if (((odd_and_frac >= 2) && (odd_and_frac <= ODD_AND_FRAC)) ||
				   ((fract_cnt == 1) && (divider == 2)))
		*frac_wf_f = 1;
	else
		*frac_wf_f = 0;

	if ((odd_and_frac >= 1) && (odd_and_frac <= EVEN_AND_FRAC))
		*frac_wf_r = 1;
	else
		*frac_wf_r = 0;

	return 0;
}
/* End of Vivado based code */

/*
 * Calculate necessary values for setting duty cycle in normal mode
 *
 * divider:		frequency divider value for calculating the phase
 * high_duty:		desired duty cycle in percent
 * pointers:		function return values, used for setting desired duty
 *
 */
static int litex_clk_calc_duty_normal(int divider, int high_duty, u8 *edge,
						   u8 *high_time, u8 *low_time,
						   u8 *frac_wf_r, u8 *frac_wf_f)
{
	int synthetic_duty, delta_d, min_d;
	u32 high_time_it, ht_aprox, edge_it;

	min_d = INT_MAX;
	/* check if duty is available to set */
	ht_aprox = high_duty * divider;
	if (ht_aprox > ((HIGH_LOW_TIME_REG_MAX * 100) + 50) ||
					(divider * 100) - ht_aprox >
					((HIGH_LOW_TIME_REG_MAX * 100) + 50))
		return -EINVAL;

	/* to prevent high_time == 0 or low_time == 0 */
	for (high_time_it = 1; high_time_it < divider; high_time_it++) {
		for (edge_it = 0; edge_it < 2; edge_it++) {
			synthetic_duty = (high_time_it * 100 + 50 * edge_it) /
								divider;
			delta_d = abs(synthetic_duty - high_duty);
			/* check if low_time won't be above acceptable range */
			if (delta_d < min_d && (divider - high_time_it) <=
						    HIGH_LOW_TIME_REG_MAX) {
				min_d = delta_d;
				*high_time = high_time_it;
				*low_time = divider - *high_time;
				*edge = edge_it;
			}
		}
	}
	/*
	 * Calculating values in normal mode,
	 * clear control bits of fractional part
	 */

	*frac_wf_f = 0;
	*frac_wf_r = 0;

	return 0;
}

/* Calculates duty cycle for given ratio in percent, 1% accuracy */
static inline int litex_clk_calc_duty_percent(struct clk_hw *hw,
					      struct clk_duty *duty)
{
	u32 div, duty_ratio, ht;

	ht = duty->num;
	div = duty->den;
	duty_ratio = ht * 10000 / div;

	return litex_round(duty_ratio, 100);
}

/* Calculates duty high_time for given divider and ratio */
static inline int litex_clk_calc_duty_high_time(struct clk_hw *hw,
						struct clk_duty *duty,
						   u32 divider)
{
	u32 high_duty;

	high_duty = litex_clk_calc_duty_percent(hw, duty) * divider;

	return litex_round(high_duty, 100);
}

/* Returns accurate duty ratio of given clkout*/
int litex_clk_get_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u32 divider;
	u16 clkout_reg1, clkout_reg2;
	u8 clkout_nr, high_time, edge, no_cnt, frac_en, frac_cnt;

	clkout_nr = l_clkout->id;
	/* Check if divider is off */
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2,
						      &clkout_reg2);
	if (ret != 0)
		return ret;
	edge = (clkout_reg2 >> EDGE_POS) & EDGE_MASK;
	no_cnt = (clkout_reg2 >> NO_CNT_POS) & NO_CNT_MASK;
	frac_en = (clkout_reg2 >> FRAC_EN_POS) & FRAC_EN_MASK;
	frac_cnt = (clkout_reg2 >> FRAC_POS) & FRAC_MASK;

	/* get duty 50% when divider is off or fractional is enabled */
	if (no_cnt || (frac_en && frac_cnt)) {
		duty->num = 1;
		duty->den = 2;
		return 0;
	}

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1,
						      &clkout_reg1);
	if (ret != 0)
		return ret;
	divider = clkout_reg1 & HL_TIME_MASK;
	high_time = (clkout_reg1 >> HIGH_TIME_POS) & HL_TIME_MASK;
	divider += high_time;

	/* Scaling to consider edge control bit */
	duty->num = high_time * 10 + edge * 5;
	duty->den = (divider + edge) * 10;

	return 0;
}

/* Set duty cycle with given ratio */
int litex_clk_set_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	u32 divider, fract_cnt, high_duty;
	u16 bitset1, bitset2;
	u8 frac_wf_r, frac_wf_f, frac_en, no_cnt,
		edge, high_time, low_time, clkout_nr;

	clkout_nr = l_clkout->id;
	edge = 0, high_time = 0, low_time = 0, frac_en = 0,
	no_cnt = 0, frac_wf_r = 0, frac_wf_f = 0;
	high_duty = litex_clk_calc_duty_percent(hw, duty);

	litex_clk_get_clkout_divider(hw, &divider, &fract_cnt);

	if (fract_cnt == 0) {
		int ret;

		ret = litex_clk_calc_duty_normal(divider, high_duty, &edge,
						     &high_time, &low_time,
						    &frac_wf_r, &frac_wf_f);
		if (ret != 0) {
			pr_err("CLKOUT%d: cannot set %d%% duty cycle for that frequency",
				clkout_nr, high_duty);
			return ret;
		}
	} else if (high_duty == 50) {
		return 0;
	} else {
		pr_err("CLKOUT%d: cannot set duty cycle when fractional divider enabled!",
								clkout_nr);
		return -EACCES;
	}

	bitset1 = (high_time << HIGH_TIME_POS) | low_time;
	bitset2 = (edge << EDGE_POS);

	pr_debug("SET DUTY CYCLE: divider:%d fract_cnt:%d frac_wf_r:%d frac_wf_f:%d frac_en:%d no_cnt:%d edge:%d high_time:%d low_time:%d bitset1: 0x%x bitset2: 0x%x",
		divider, fract_cnt, frac_wf_r, frac_wf_f, frac_en,
		no_cnt, edge, high_time, low_time, bitset1, bitset2);

	return litex_clk_set_clock(hw, clkout_nr, REG1_DUTY_MASK, bitset1,
						  REG2_DUTY_MASK, bitset2);
}

/*
 *	Phase
 */

/*
 * Calculate necessary values for setting phase in fractional mode
 *
 * clk_hw:		hardware specific clock structure
 * divider:		frequency divider value for calculating the phase
 * fract_cnt:		fractional frequency divider value, if not for
 *			CLKOUT0 - set 0
 * phase_offset:	desired phase in angle degrees
 * pointers:		function return values, used for setting desired phase
 *
 * This function is based on Xilinx Unified Simulation Library Component
 * Advanced Mixed Mode Clock Manager (MMCM) taken from Xilinx Vivado
 * found in: <Vivado-dir>/data/verilog/src/unisims/MMCME2_ADV.v
 *
 */
static int litex_clk_calc_phase_fract(struct clk_hw *hw, u32 divider,
					u32 fract_cnt, u32 phase_offset,
					u32 *phase_mux, u32 *delay_time,
					u32 *phase_mux_f, u32 *period_offset)
{
	int phase, dt_calc, dt,
		a_per_in_octets, a_phase_in_cycles,
		pm_rise_frac, pm_rise_frac_filtered,
		pm_fall, pm_fall_frac, pm_fall_frac_filtered;

	phase = phase_offset * 1000;
	a_per_in_octets = (8 * divider) + fract_cnt;
	a_phase_in_cycles = (phase + 10) * a_per_in_octets / 360000;

	if ((a_phase_in_cycles & FULL_BYTE) != 0)
		pm_rise_frac = (a_phase_in_cycles & PHASE_MUX_MAX);
	else
		pm_rise_frac = 0;

	dt_calc = a_phase_in_cycles / 8;
	dt = dt_calc & FULL_BYTE;

	pm_rise_frac_filtered = pm_rise_frac;
	if (pm_rise_frac >= 8)
		pm_rise_frac_filtered -= 8;

	pm_fall = ((divider % 2) << 2) + ((fract_cnt >> 1) & TWO_LSBITS);
	pm_fall_frac = pm_fall + pm_rise_frac;
	pm_fall_frac_filtered = pm_fall + pm_rise_frac -
					 (pm_fall_frac & F_FRAC_MASK);

	/* keep only the lowest bits to fit in control registers bit groups */
	*delay_time = dt % (1 << 6);
	*phase_mux = pm_rise_frac_filtered % (1 << 3);
	*phase_mux_f = pm_fall_frac_filtered % (1 << 3);

	return 0;
}
/* End of Vivado based code */

/*
 * Calculate necessary values for setting phase in normal mode
 *
 * clk_hw:		hardware specific clock structure
 * divider:		frequency divider value for calculating the phase
 * fract_cnt:		fractional frequency divider value, if not for
 *			CLKOUT0 - set 0
 * phase_offset:	desired phase in angle degrees
 * pointers:		function return values, used for setting desired phase
 *
 */
static int litex_clk_calc_phase_normal(struct clk_hw *hw, u32 divider,
					u32 fract_cnt, u32 phase_offset,
					u32 *phase_mux, u32 *delay_time,
					u32 *phase_mux_f, u32 *period_offset)
{
	/* ns unit */
	u32 global_period, clkout_period, post_glob_div_f;

	post_glob_div_f = litex_clk_get_real_global_frequency(hw);
	global_period = NS_IN_SEC / post_glob_div_f;
	clkout_period = global_period * divider;

	if (phase_offset != 0) {
		int synthetic_phase, delta_p, min_p;
		u8 d_t, p_m;

		*period_offset = litex_round(clkout_period * (*period_offset),
								10000);

		if (*period_offset / global_period > DELAY_TIME_MAX)
			return -EINVAL;

		min_p = INT_MAX;
		/* Delay_time: (0-63) */
		for (d_t = 0; d_t <= DELAY_TIME_MAX; d_t++) {
			/* phase_mux: (0-7) */
			for (p_m = 0; p_m <= PHASE_MUX_MAX; p_m++) {
				synthetic_phase = (d_t * global_period) +
				((p_m * ((global_period * 100) / 8) / 100));

				delta_p = abs(synthetic_phase - *period_offset);
				if (delta_p < min_p) {
					min_p = delta_p;
					*phase_mux = p_m;
					*delay_time = d_t;
				}
			}
		}
	} else {
		/* Don't change phase offset*/
		*phase_mux = 0;
		*delay_time = 0;
	}
/* Calculating values in normal mode, fractional control bits need to be zero*/
		*phase_mux_f = 0;

	return 0;
}

/*
 * Convert phase offset to positive lower than 360 deg.
 * and calculate period in nanoseconds
 *
 */
static int litex_clk_prepare_phase(int *phase_offset, u32 *period_offset)
{
	*phase_offset = *phase_offset % 360;

	if (*phase_offset < 0)
		*phase_offset += 360;

	*period_offset = ((*phase_offset * 10000) / 360);

	return 0;
}

/*
 * Calculate necessary values for setting phase
 *
 * clk_hw:		hardware specific clock structure
 * divider:		frequency divider value for calculating the phase
 * fract_cnt:		fractional frequency divider value, if not for
 *			CLKOUT0 - set 0
 * clkout_nr:		clock output number
 * phase_offset:	desired phase in angle degrees
 * pointers:		function return values, used for setting desired phase
 *
 */
static int litex_clk_calc_phase(struct clk_hw *clk_hw,
					u32 divider, u32 fract_cnt,
					u32 clkout_nr, int phase_offset,
					u32 *phase_mux, u32 *delay_time,
					u32 *phase_mux_f, u32 *period_offset)
{
	int ret;

	litex_clk_prepare_phase(&phase_offset, period_offset);

	if (clkout_nr == 0 && fract_cnt > 0)
		ret = litex_clk_calc_phase_fract(clk_hw, divider, fract_cnt,
						phase_offset,
						phase_mux, delay_time,
						phase_mux_f, period_offset);
	else
		ret = litex_clk_calc_phase_normal(clk_hw, divider, fract_cnt,
						phase_offset,
						phase_mux, delay_time,
						phase_mux_f, period_offset);
	return ret;
}

/* Returns phase-specific values of given clock output */
static int litex_clk_get_phase_data(struct clk_hw *hw, u8 *phase_mux,
				       u8 *delay_time, u8 *phase_mux_f)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u16 r1, r2;
	u8 clkout_nr = l_clkout->id;

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1, &r1);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2, &r2);
	if (ret != 0)
		return ret;
	*phase_mux = (r1 >> PHASE_MUX_POS) & PHASE_MUX_MASK;
	*delay_time = (r2 >> DELAY_TIME_POS) & HL_TIME_MASK;

	if (clkout_nr == 0) {
		ret = litex_clk_get_DO(hw, CLKOUT5_REG2, &r2);
		if (ret != 0)
			return ret;
		*phase_mux_f = (r2 >> PHASE_MUX_F_POS) & PHASE_MUX_F_MASK;
	} else {
		*phase_mux_f = 0;
	}

	return 0;
}

/* Returns phase of given clock output in degrees */
int litex_clk_get_phase(struct clk_hw *hw)
{
	u8 phase_mux = 0, delay_time = 0, phase_mux_f;
	u32 divider, fract_cnt, post_glob_div_f, pm;
	/* ns unit */
	u32 global_period, clkout_period, period;

	litex_clk_get_phase_data(hw, &phase_mux, &delay_time, &phase_mux_f);
	litex_clk_get_clkout_divider(hw, &divider, &fract_cnt);

	post_glob_div_f = litex_clk_get_real_global_frequency(hw);
	global_period = NS_IN_SEC / post_glob_div_f;
	clkout_period = global_period * divider;

	pm = (phase_mux * global_period * 1000) / PHASE_MUX_RES_FACTOR;
	pm = litex_round(pm, 1000);

	period = delay_time * global_period + pm;

	period = period * 1000 / clkout_period;
	period = period * 360;

	return litex_round(period, 1000);
}

/* Sets phase given in degrees on given clock output */
int litex_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	u32 phase_mux, divider, fract_cnt,
	    phase_mux_f, delay_time, period_offset;
	u16 bitset1, bitset2, reg2_mask;
	u8 clkout_nr, edge, high_time, low_time, frac_wf_r, frac_wf_f;
	int ret;

	reg2_mask = REG2_PHASE_MASK;
	phase_mux = 0;
	delay_time = 0;
	clkout_nr = l_clkout->id;
	litex_clk_get_clkout_divider(hw, &divider, &fract_cnt);
	if (fract_cnt > 0 && degrees != 0) {
		pr_err("CLKOUT%d: cannot set phase on that frequency",
								clkout_nr);
		return -ENOSYS;
	}
	ret = litex_clk_calc_phase(hw, divider, fract_cnt, clkout_nr, degrees,
						&phase_mux, &delay_time,
						&phase_mux_f, &period_offset);
	if (ret != 0) {
		pr_err("CLKOUT%d: phase offset %d deg is too high for given frequency",
							clkout_nr, degrees);
		return ret;
	}

	bitset1 = (phase_mux << PHASE_MUX_POS);
	bitset2 = (delay_time << DELAY_TIME_POS);

	if (clkout_nr == 0 && fract_cnt > 0) {
		u16 clkout5_reg2_bitset;

		litex_clk_calc_duty_fract(divider, 50, fract_cnt,
						&edge, &high_time, &low_time,
						&frac_wf_r, &frac_wf_f);
		bitset2 |= (fract_cnt << FRAC_POS)	|
			   (1 << FRAC_EN_POS)		|
			   (frac_wf_r << FRAC_WF_R_POS);

		clkout5_reg2_bitset = (phase_mux_f << PHASE_MUX_F_POS)	|
				      (frac_wf_f << FRAC_WF_F_POS);

		ret = litex_clk_change_value(hw, CLKOUT5_FRAC_MASK,
					   clkout5_reg2_bitset,
					   CLKOUT5_REG2);
		if (ret != 0)
			return ret;
		reg2_mask = REG2_PHASE_F_MASK;
	}

	pr_debug("SET PHASE: phase_mux:%d phase_mux_f:%d delay_time:%d bitset1: 0x%x bitset2: 0x%x",
		phase_mux, phase_mux_f, delay_time, bitset1, bitset2);

	return litex_clk_set_clock(hw, clkout_nr, REG1_PHASE_MASK, bitset1,
						 reg2_mask, bitset2);
}

/*
 *	Frequency
 */

/*
 * Calculate necessary values for setting frequency in fractional mode
 *
 * clk_hw:		hardware specific clock structure
 * freq:		desired frequency in Hz
 * pointers:		function return values, used for setting
 *			desired frequency
 */
static int litex_clk_calc_freq_fract(struct clk_hw *clk_hw, u32 freq,
						u32 *divider, u8 *frac_en,
						u8 *no_cnt, u32 *fract_cnt)
{
	int delta_f, synthetic_freq, min_f;
	u32 post_glob_div_f, div, f_cnt, fract;

	post_glob_div_f = litex_clk_get_real_global_frequency(clk_hw);

	delta_f = 0;
	synthetic_freq = post_glob_div_f;
	min_f = abs(synthetic_freq - freq);
	*divider = 1;
	/* div is integer divider * 1000 to keep precision */
	for (div = MIN_DIVIDER_F; div <= MAX_DIVIDER_F; div += 1000) {
		/*
		 * f_cnt is fractional part of divider * 1000 incremented
		 * by minimal fractional divider step to keep precision
		 * and find best divider
		 */
		for (f_cnt = 0, fract = 0; fract < 1000;
				fract += FRACT_DIV_RES, f_cnt++) {

			synthetic_freq = (post_glob_div_f / (div + fract)) *
								       1000;
			delta_f = abs(synthetic_freq - freq);

			if (delta_f < min_f) {
				min_f = delta_f;
				*divider = div / 1000;
				*fract_cnt = f_cnt;
			}
		}
	}
	if (*fract_cnt == 0) {
		*frac_en = 0;
		if (*divider == 1)
			*no_cnt = 1;
		else
			*no_cnt = 0;
	} else {
		*frac_en = 1;
		*no_cnt = 0;
	}

	return 0;
}

/*
 * Calculate necessary values for setting frequency in normal mode
 *
 * clk_hw:		hardware specific clock structure
 * freq:		desired frequency in Hz
 * pointers:		function return values, used for setting
 *			desired frequency
 *
 */
static int litex_clk_calc_freq_normal(struct clk_hw *clk_hw, u32 freq,
						u32 *divider, u8 *frac_en,
						u8 *no_cnt, u32 *fract_cnt)
{
	int delta_f, synthetic_freq, min_f;
	u32 post_glob_div_f, div;

	post_glob_div_f = litex_clk_get_real_global_frequency(clk_hw);
	synthetic_freq = post_glob_div_f;
	min_f = abs(synthetic_freq - freq);
	*divider = 1;
	delta_f = 0;

	for (div = MIN_DIVIDER; div <= MAX_DIVIDER; div++) {
		synthetic_freq = post_glob_div_f / div;
		delta_f = abs(synthetic_freq - freq);
		if (delta_f < min_f) {
			min_f = delta_f;
			*divider = div;
		}
	}
	if (*divider == 1)
		*no_cnt = 1;
	else
		*no_cnt = 0;

	*frac_en = 0;
	*fract_cnt = 0;

	return 0;
}

/* Returns rate in Hz */
static inline unsigned long litex_clk_calc_rate(struct clk_hw *hw, u32 g_mul,
							u32 g_div, u32 div,
							u32 fract_cnt)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);

	return (l_clkout->sys_clk_freq * g_mul /
	      ((div * 1000 + (fract_cnt * FRACT_DIV_RES)) * g_div)) * 1000;
}

/*
 * Calculate necessary values for setting frequency
 *
 * clk_hw:		hardware specific clock structure
 * rate:		desired frequency in Hz
 * clkout_nr:		clock output number
 * pointers:		function return values, used for setting
 *			desired frequency
 *
 */
static void litex_clk_calc_dividers(struct clk_hw *hw, u32 rate, u8 clkout_nr,
					u32 *divider, u32 *fract_cnt,
					u8 *frac_wf_r, u8 *frac_wf_f,
					u8 *frac_en, u8 *no_cnt, u8 *edge,
					u8 *high_time, u8 *low_time)
{
	u32 high_duty;
	struct clk_duty duty;

	litex_clk_get_duty_cycle(hw, &duty);
	high_duty = litex_clk_calc_duty_percent(hw, &duty);

	if (clkout_nr == 0) {
		litex_clk_calc_freq_fract(hw, rate, divider, frac_en, no_cnt,
								   fract_cnt);
		if (*fract_cnt > 0)
			litex_clk_calc_duty_fract(*divider, high_duty,
						  *fract_cnt, edge,
						   high_time, low_time,
						   frac_wf_r, frac_wf_f);
		else
			litex_clk_calc_duty_normal(*divider, high_duty, edge,
						    high_time, low_time,
						    frac_wf_r, frac_wf_f);
	} else {

		litex_clk_calc_freq_normal(hw, rate, divider, frac_en,
						     no_cnt, fract_cnt);

		litex_clk_calc_duty_normal(*divider, high_duty, edge,
						     high_time, low_time,
						     frac_wf_r, frac_wf_f);
	}
}

/* Returns rate of given CLKOUT, parent_rate ignored */
unsigned long litex_clk_recalc_rate(struct clk_hw *hw,
				    unsigned long parent_rate)
{
	u32 multiplier, divider, clkout_div, fract_cnt;

	LITEX_UNUSED(parent_rate);

	litex_clk_get_global_divider(hw, &divider, &multiplier);
	litex_clk_get_clkout_divider(hw, &clkout_div, &fract_cnt);
	return litex_clk_calc_rate(hw, multiplier, divider, clkout_div,
							     fract_cnt);
}

int litex_clk_check_rate_range(struct clk_hw *hw, unsigned long rate)
{
	u32 f_max, f_min, gf;

	gf = litex_clk_get_real_global_frequency(hw);
	f_max = gf / 2;
	f_min = gf / MAX_DIVIDER;
	/* margin of 1kHz */
	if ((rate > f_max + KHZ) | (rate < f_min - KHZ))
		return -EINVAL;

	return 0;
}

/* Returns closest available clock rate in Hz, parent_rate ignored */
long litex_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					     unsigned long *parent_rate)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	u32 divider, fract_cnt, g_divider, g_multiplier;
	u8 frac_wf_r, frac_wf_f, frac_en, no_cnt,
		edge, high_time, low_time, clkout_nr;

	if (litex_clk_check_rate_range(hw, rate))
		return -EINVAL;

	LITEX_UNUSED(parent_rate);
	clkout_nr = l_clkout->id;

	litex_clk_calc_dividers(hw, rate, clkout_nr, &divider, &fract_cnt,
				&frac_wf_r, &frac_wf_f, &frac_en,
				&no_cnt, &edge, &high_time, &low_time);
	litex_clk_get_global_divider(hw, &g_divider, &g_multiplier);

	return litex_clk_calc_rate(hw, g_multiplier, g_divider, divider,
								fract_cnt);
}

/* Set closest available clock rate in Hz, parent_rate ignored */
int litex_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					  unsigned long parent_rate)
{
	struct litex_clk_clkout *l_clkout = clk_hw_to_litex_clk_clkout(hw);
	u16 bitset1, bitset2;
	u32 divider, fract_cnt;
	u8 frac_wf_r, frac_wf_f, frac_en, no_cnt,
		edge, high_time, low_time, clkout_nr;
	int ret;

	LITEX_UNUSED(parent_rate);
	clkout_nr = l_clkout->id;

	pr_debug("set_rate: %lu", rate);
	if (litex_clk_check_rate_range(hw, rate)) {
		pr_err("CLKOUT%d: cannot set %luHz, frequency not in available range",
							clkout_nr, rate);
		return -EINVAL;
	}

	litex_clk_calc_dividers(hw, rate, clkout_nr, &divider, &fract_cnt,
				&frac_wf_r, &frac_wf_f, &frac_en, &no_cnt,
					    &edge, &high_time, &low_time);

	bitset1 = (high_time << HIGH_TIME_POS)	|
		  (low_time << LOW_TIME_POS);

	bitset2 = (fract_cnt << FRAC_POS)	|
		  (frac_en << FRAC_EN_POS)	|
		  (frac_wf_r << FRAC_WF_R_POS)	|
		  (edge << EDGE_POS)		|
		  (no_cnt << NO_CNT_POS);
	if (frac_en != 0) {
		u32 phase_mux, delay_time, phase_mux_f,
		    phase_offset, period_offset;

		phase_offset = 0;
		litex_clk_calc_phase_fract(hw, divider, fract_cnt, phase_offset,
					       &phase_mux, &delay_time,
					       &phase_mux_f, &period_offset);

		bitset1 |= (phase_mux << PHASE_MUX_POS);
		bitset2 |= (delay_time << DELAY_TIME_POS);

		if (frac_wf_f > 0 || phase_mux_f > 0) {
			u16 clkout5_reg2_bitset = 0;

			clkout5_reg2_bitset = (phase_mux_f << PHASE_MUX_F_POS) |
					      (frac_wf_f << FRAC_WF_F_POS);

			ret = litex_clk_change_value(hw, CLKOUT5_FRAC_MASK,
				    clkout5_reg2_bitset, CLKOUT5_REG2);
			if (ret != 0)
				return ret;
		}
	}

	pr_debug("SET RATE: divider:%d fract_cnt:%d frac_wf_r:%d frac_wf_f:%d frac_en:%d no_cnt:%d edge:%d high_time:%d low_time:%d bitset1: 0x%x bitset2: 0x%x",
		divider, fract_cnt, frac_wf_r, frac_wf_f, frac_en,
		no_cnt, edge, high_time, low_time, bitset1, bitset2);

	return litex_clk_set_clock(hw, clkout_nr, REG1_FREQ_MASK, bitset1,
						  REG2_FREQ_MASK, bitset2);
}

/* Prepare clock output */
int litex_clk_prepare(struct clk_hw *hw)
{
	struct litex_clk_clkout *l_clkout;
	struct clk_duty duty;
	u32 freq, phase;

	l_clkout = clk_hw_to_litex_clk_clkout(hw);
	freq = l_clkout->default_freq;
	phase = l_clkout->default_phase;
	duty.num = l_clkout->default_duty_num;
	duty.den = l_clkout->default_duty_den;

	litex_clk_set_rate(hw, freq, 0);
	litex_clk_set_duty_cycle(hw, &duty);
	litex_clk_set_phase(hw, phase);
	return 0;
}

/* Unrepare clock output */
void litex_clk_unprepare(struct clk_hw *hw)
{
	struct clk_duty duty;

	duty.num = 1;
	duty.den = 2;
	litex_clk_set_rate(hw, 100000000, 0);
	litex_clk_set_duty_cycle(hw, &duty);
	litex_clk_set_phase(hw, 0);
}

/* Enable Clock Control MMCM module */
int litex_clk_enable(struct clk_hw *hw)
{
	litex_clk_assert_reg(hw, DRP_RESET);
	return 0;
}

/* Disable Clock Control MMCM module */
void litex_clk_disable(struct clk_hw *hw)
{
	litex_clk_deassert_reg(hw, DRP_RESET);
}

/* Set default clock value from device tree for given clkout*/
static int litex_clk_set_def_clkout(struct clk_hw *hw, u32 freq, u32 phase,
							struct clk_duty *duty)
{
	int ret;

	ret = litex_clk_set_rate(hw, freq, 0);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_duty_cycle(hw, duty);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_phase(hw, phase);
	if (ret != 0)
		return ret;
	return 0;
}

static const struct clk_ops litex_clk_ops = {
	.prepare		=	litex_clk_prepare,
	.unprepare		=	litex_clk_unprepare,
	.enable			=	litex_clk_enable,
	.disable		=	litex_clk_disable,
	.recalc_rate		=	litex_clk_recalc_rate,
	.round_rate		=	litex_clk_round_rate,
	.set_rate		=	litex_clk_set_rate,
	.get_phase		=	litex_clk_get_phase,
	.set_phase		=	litex_clk_set_phase,
	.get_duty_cycle		=	litex_clk_get_duty_cycle,
	.set_duty_cycle		=	litex_clk_set_duty_cycle,
};

static const struct of_device_id litex_of_match[] = {
	{.compatible = "litex,clk"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static int litex_clk_read_clkout_dts(struct device_node *child,
				     struct litex_clk_clkout *clkout)
{
	int ret;
	u32 dt_num, dt_freq, dt_phase, dt_duty_num,
	    dt_duty_den, f_max, f_min, gf;

	gf = litex_clk_get_expctd_global_frequency(clkout->sys_clk_freq);
	f_max = gf / 2;
	f_min = gf / MAX_DIVIDER;

	ret = of_property_read_u32(child, "reg", &dt_num);
	if (ret != 0)
		return -ret;

	if (dt_num > CLKOUT_MAX) {
		pr_err("%s: Invalid CLKOUT index!", child->name);
		return -EINVAL;
	}

	ret = of_property_read_u32(child, "litex,clock-frequency", &dt_freq);
	if (ret != 0)
		return -ret;

	if (dt_freq > f_max || dt_freq < f_min) {
		pr_err("%s: Invalid default frequency!(%d), MIN/MAX (%d/%d)Hz",
			child->name, dt_freq, f_min, f_max);
		return -EINVAL;
	}

	ret = of_property_read_u32(child, "litex,clock-phase", &dt_phase);
	if (ret != 0)
		return -ret;

	ret = of_property_read_u32(child, "litex,clock-duty-den", &dt_duty_den);
	if (ret != 0)
		return -ret;

	ret = of_property_read_u32(child, "litex,clock-duty-num", &dt_duty_num);
	if (ret != 0)
		return -ret;

	if (dt_duty_den <= 0 || dt_duty_num > dt_duty_den) {
		pr_err("%s: Invalid default duty! %d/%d", child->name,
						dt_duty_num, dt_duty_den);
		return -EINVAL;
	}

	clkout->id = dt_num;
	clkout->default_freq = dt_freq;
	clkout->default_phase = dt_phase;
	clkout->default_duty_num = dt_duty_num;
	clkout->default_duty_den = dt_duty_den;

	return 0;
}

static int litex_clk_get_timeouts(struct device_node *node, int *dt_lock_timeout,
							    int *dt_drdy_timeout)
{
	int ret;

	/* Read wait_lock timeout from device property*/
	ret = of_property_read_u32(node, "litex,lock-timeout", dt_lock_timeout);
	if (ret < 0) {
		pr_err("No litex,lock_timeout entry in the dts file\n");
		return -ENODEV;
	}
	if (*dt_lock_timeout < 1) {
		pr_err("LiteX CLK driver cannot wait for time bellow ca. 1ms\n");
		return -EINVAL;
	}

	/* Read wait_drdy timeout from device property*/
	ret = of_property_read_u32(node, "litex,drdy-timeout", dt_drdy_timeout);
	if (ret < 0) {
		pr_err("No litex,lock_drdy entry in the dts file\n");
		return -ENODEV;
	}
	if (*dt_drdy_timeout < 1) {
		pr_err("LiteX CLK driver cannot wait for time bellow ca. 1ms\n");
		return -EINVAL;
	}

	return 0;
}

static int litex_clk_init_clkouts(struct device *dev,
				  struct device_node *node,
				  struct litex_clk_device *l_dev)
{
	struct device_node *child_node;
	struct clk_hw *hw;
	struct clk_duty duty;
	u32 dt_lock_timeout, dt_drdy_timeout;
	int i = 0, ret;

	ret = litex_clk_get_timeouts(node, &dt_lock_timeout, &dt_drdy_timeout);
	if (ret != 0)
		return ret;

	for_each_child_of_node(node, child_node) {
		struct litex_clk_clkout *clkout;
		struct clk_init_data clkout_init;

		clkout = &l_dev->clkouts[i];
		clkout->is_clkout = true;

		clkout->lock_timeout = dt_lock_timeout;
		clkout->drdy_timeout = dt_drdy_timeout;
		clkout->sys_clk_freq = l_dev->sys_clk_freq;

		ret = litex_clk_read_clkout_dts(child_node, clkout);
		if (ret != 0)
			return ret;

		clkout_init.name = child_node->name;
		clkout_init.ops = &litex_clk_ops;
		clkout_init.num_parents = 0;
		clkout_init.flags = CLK_SET_RATE_UNGATE | CLK_GET_RATE_NOCACHE;
		clkout->clk_hw.init = &clkout_init;
		clkout->base = l_dev->base;

		duty.num = clkout->default_duty_num;
		duty.den = clkout->default_duty_den;

		/* set global divider and multiplier once */
		if (i == 0) {
			hw = &l_dev->clkouts[i].clk_hw;
			litex_clk_enable(hw);
			ret = litex_clk_set_dm(hw);
			litex_clk_disable(hw);
			if (ret != 0)
				return ret;
		}

		ret = litex_clk_set_def_clkout(&clkout->clk_hw,
						clkout->default_freq,
						clkout->default_phase, &duty);
		ret = devm_clk_hw_register(dev, &clkout->clk_hw);
		if (ret != 0) {
			pr_err("devm_clk_hw_register failure, %s ret: %d\n",
						child_node->name, ret);
			return ret;
		}

		ret = of_clk_add_hw_provider(child_node, of_clk_hw_simple_get,
							     &clkout->clk_hw);
		if (ret != 0)
			return ret;
		i++;
	}
	return 0;
}

static int litex_clk_read_global_dts(struct device *dev,
				     struct device_node *node,
				     struct litex_clk_device *l_dev)
{
	struct device_node *child_node;
	struct litex_clk_clkout *l_clkout;
	u32 dt_nclkout, dt_lock_timeout, dt_drdy_timeout, dt_sys_clk_freq;
	int i = 0, ret;

	ret = of_property_read_u32(node, "litex,sys-clock-frequency",
						&dt_sys_clk_freq);
	if (ret < 0) {
		pr_err("No clock-frequency entry in the dts file\n");
		return -ENODEV;
	}
	l_dev->sys_clk_freq = dt_sys_clk_freq;

	/* Read cnt of CLKOUTs from device property*/
	ret = of_property_read_u32(node, "litex,nclkout", &dt_nclkout);
	if (ret < 0) {
		pr_err("No litex,nclkout entry in the dts file\n");
		return -ENODEV;
	}
	if (dt_nclkout > CLKOUT_MAX) {
		pr_err("LiteX CLK driver cannot use more than %d clock outputs\n",
								CLKOUT_MAX);
		return -EINVAL;
	}
	l_dev->nclkout = dt_nclkout;

	/* count existing CLKOUTs */
	for_each_child_of_node(node, child_node)
		i++;
	if (i != dt_nclkout) {
		pr_err("nclkout(%d) not matching actual CLKOUT count(%d)!",
								dt_nclkout, i);
		return -EINVAL;
	}

	l_dev->clkouts = devm_kzalloc(dev, sizeof(*l_clkout) * i, GFP_KERNEL);
	if (!l_dev->clkouts) {
		pr_err("CLKOUT memory allocation failure!");
		return -ENOMEM;
	}

	ret = litex_clk_get_timeouts(node, &dt_lock_timeout, &dt_drdy_timeout);
	if (ret != 0)
		return ret;
	l_dev->lock_timeout = dt_lock_timeout;
	l_dev->drdy_timeout = dt_drdy_timeout;

	return 0;
}

static int litex_clk_init_glob_clk(struct device *dev,
				   struct device_node *node,
				   struct litex_clk_device *l_dev)
{
	struct clk_init_data init;
	struct clk_hw *hw;
	int ret;

	init.name = node->name;
	init.ops = &litex_clk_ops;
	init.flags = CLK_SET_RATE_UNGATE;

	hw = &l_dev->clk_hw;
	hw->init = &init;

	ret = devm_clk_hw_register(dev, hw);
	if (ret != 0) {
		pr_err("devm_clk_hw_register failure, ret: %d\n", ret);
		return ret;
	}

	ret = of_clk_add_hw_provider(node, of_clk_hw_simple_get, hw);
	if (ret != 0) {
		pr_err("of_clk_add_hw_provider failure, ret: %d", ret);
		return ret;
	}

	/* Power on MMCM module */
	ret = litex_clk_change_value(hw, FULL_REG_16, FULL_REG_16, POWER_REG);
	if (ret != 0) {
		pr_err("MMCM initialization failure, ret: %d", ret);
		return ret;
	}

	return 0;
}

static int litex_clk_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device *dev;
	struct device_node *node;
	struct litex_clk_device *litex_clk_device;
	struct resource *res;
	int ret;

	dev = &pdev->dev;
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	id = of_match_node(litex_of_match, node);
	if (!id)
		return -ENODEV;

	litex_clk_device = devm_kzalloc(dev, sizeof(*litex_clk_device),
								GFP_KERNEL);
	if (IS_ERR_OR_NULL(litex_clk_device))
		return -ENOMEM;
	litex_clk_device->is_clkout = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res))
		return -EBUSY;

	litex_clk_device->base = devm_of_iomap(dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(litex_clk_device->base))
		return -EIO;

	ret = litex_clk_read_global_dts(dev, node, litex_clk_device);
	if (ret != 0)
		return ret;

	ret = litex_clk_init_clkouts(dev, node, litex_clk_device);
	if (ret != 0)
		return ret;

	ret = litex_clk_init_glob_clk(dev, node, litex_clk_device);
	if (ret != 0)
		return ret;

	pr_info("litex clk control driver initialized");
	return 0;
}

static int litex_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static struct platform_driver litex_clk_driver = {
	.driver = {
		.name = "litex-clk",
		.of_match_table = of_match_ptr(litex_of_match),
	},
	.probe = litex_clk_probe,
	.remove = litex_clk_remove,
};

module_platform_driver(litex_clk_driver);
MODULE_DESCRIPTION("LiteX clock driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
