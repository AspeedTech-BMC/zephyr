/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <xtensa/xtruntime.h>
#include <irq_nextlevel.h>
#include <xtensa/hal.h>
#include <init.h>

#include "soc.h"

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(soc);

static uint32_t ref_clk_freq;

#define CAVS_INTC_NODE(n) DT_INST(n, intel_cavs_intc)

void z_soc_irq_enable(uint32_t irq)
{
	const struct device *dev_cavs, *dev_ictl;

	switch (XTENSA_IRQ_NUMBER(irq)) {
	case DT_IRQN(CAVS_INTC_NODE(0)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(0)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(1)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(1)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(2)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(2)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(3)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(3)));
		break;
	default:
		/* regular interrupt */
		z_xtensa_irq_enable(XTENSA_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_cavs) {
		LOG_DBG("board: CAVS device binding failed");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in either CAVS interrupt logic or DW interrupt controller
	 */
	z_xtensa_irq_enable(XTENSA_IRQ_NUMBER(irq));

	switch (CAVS_IRQ_NUMBER(irq)) {
	case DW_ICTL_IRQ_CAVS_OFFSET:
		dev_ictl = device_get_binding(DT_LABEL(DT_INST(0, snps_designware_intc)));
		break;
	default:
		/* The source of the interrupt is in CAVS interrupt logic */
		irq_enable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_ictl) {
		LOG_DBG("board: DW intr_control device binding failed");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in DW interrupt controller
	 */
	irq_enable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

	/* Manipulate the relevant bit in the interrupt controller
	 * register as needed
	 */
	irq_enable_next_level(dev_ictl, INTR_CNTL_IRQ_NUM(irq));
}

void z_soc_irq_disable(uint32_t irq)
{
	const struct device *dev_cavs, *dev_ictl;

	switch (XTENSA_IRQ_NUMBER(irq)) {
	case DT_IRQN(CAVS_INTC_NODE(0)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(0)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(1)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(1)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(2)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(2)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(3)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(3)));
		break;
	default:
		/* regular interrupt */
		z_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_cavs) {
		LOG_DBG("board: CAVS device binding failed");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in either CAVS interrupt logic or DW interrupt controller
	 */

	switch (CAVS_IRQ_NUMBER(irq)) {
	case DW_ICTL_IRQ_CAVS_OFFSET:
		dev_ictl = device_get_binding(DT_LABEL(DT_INST(0, snps_designware_intc)));
		break;
	default:
		/* The source of the interrupt is in CAVS interrupt logic */
		irq_disable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

		/* Disable the parent IRQ if all children are disabled */
		if (!irq_is_enabled_next_level(dev_cavs)) {
			z_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		}
		return;
	}

	if (!dev_ictl) {
		LOG_DBG("board: DW intr_control device binding failed");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in DW interrupt controller.
	 * Manipulate the relevant bit in the interrupt controller
	 * register as needed
	 */
	irq_disable_next_level(dev_ictl, INTR_CNTL_IRQ_NUM(irq));

	/* Disable the parent IRQ if all children are disabled */
	if (!irq_is_enabled_next_level(dev_ictl)) {
		irq_disable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

		if (!irq_is_enabled_next_level(dev_cavs)) {
			z_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		}
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	const struct device *dev_cavs, *dev_ictl;
	int ret = -EINVAL;

	switch (XTENSA_IRQ_NUMBER(irq)) {
	case DT_IRQN(CAVS_INTC_NODE(0)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(0)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(1)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(1)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(2)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(2)));
		break;
	case DT_IRQN(CAVS_INTC_NODE(3)):
		dev_cavs = device_get_binding(DT_LABEL(CAVS_INTC_NODE(3)));
		break;
	default:
		/* regular interrupt */
		ret = z_xtensa_irq_is_enabled(XTENSA_IRQ_NUMBER(irq));
		goto out;
	}

	if (!dev_cavs) {
		LOG_DBG("board: CAVS device binding failed");
		ret = -ENODEV;
		goto out;
	}

	switch (CAVS_IRQ_NUMBER(irq)) {
	case DW_ICTL_IRQ_CAVS_OFFSET:
		dev_ictl = device_get_binding(DT_LABEL(DT_INST(0, snps_designware_intc)));
		break;
	default:
		/* The source of the interrupt is in CAVS interrupt logic */
		ret = irq_line_is_enabled_next_level(dev_cavs,
						     CAVS_IRQ_NUMBER(irq));
		goto out;
	}

	if (!dev_ictl) {
		LOG_DBG("board: DW intr_control device binding failed");
		ret = -ENODEV;
		goto out;
	}

	ret = irq_line_is_enabled_next_level(dev_ictl, INTR_CNTL_IRQ_NUM(irq));

out:
	return ret;
}

static inline void soc_set_resource_ownership(void)
{
	volatile struct soc_resource_alloc_regs *regs =
		(volatile struct soc_resource_alloc_regs *)
		SOC_RESOURCE_ALLOC_REG_BASE;
	int index;


	/* set ownership of DMA controllers and channels */
	for (index = 0; index < SOC_NUM_LPGPDMAC; index++) {
		regs->lpgpdmacxo[index] = SOC_LPGPDMAC_OWNER_DSP;
	}

	/* set ownership of I2S and DMIC controllers */
	regs->dspiopo = SOC_DSPIOP_I2S_OWNSEL_DSP |
		SOC_DSPIOP_DMIC_OWNSEL_DSP;

	/* set ownership of timestamp and M/N dividers */
	regs->geno = SOC_GENO_TIMESTAMP_OWNER_DSP |
		SOC_GENO_MNDIV_OWNER_DSP;
}

uint32_t soc_get_ref_clk_freq(void)
{
	return ref_clk_freq;
}

static inline void soc_set_audio_mclk(void)
{
#if (CONFIG_AUDIO)
	int mclk;
	volatile struct soc_mclk_control_regs *mclk_regs =
		(volatile struct soc_mclk_control_regs *)SOC_MCLK_DIV_CTRL_BASE;

	for (mclk = 0; mclk < SOC_NUM_MCLK_OUTPUTS; mclk++) {
		/*
		 * set divider to bypass mode which makes MCLK output frequency
		 * to be the same as referece clock frequency
		 */
		mclk_regs->mdivxr[mclk] = SOC_MDIVXR_SET_DIVIDER_BYPASS;
		mclk_regs->mdivctrl |= SOC_MDIVCTRL_MCLK_OUT_EN(mclk);
	}
#endif
}

static inline void soc_set_dmic_power(void)
{
#if (CONFIG_AUDIO_INTEL_DMIC)
	volatile struct soc_dmic_shim_regs *dmic_shim_regs =
		(volatile struct soc_dmic_shim_regs *)SOC_DMIC_SHIM_REG_BASE;

	/* enable power */
	dmic_shim_regs->dmiclctl |= SOC_DMIC_SHIM_DMICLCTL_SPA;

	while ((dmic_shim_regs->dmiclctl & SOC_DMIC_SHIM_DMICLCTL_CPA) == 0U) {
		/* wait for power status */
	}
#endif
}

static inline void soc_set_gna_power(void)
{
#if (CONFIG_INTEL_GNA)
	volatile struct soc_global_regs *regs =
		(volatile struct soc_global_regs *)SOC_S1000_GLB_CTRL_BASE;

	/* power on GNA block */
	regs->gna_power_control |= SOC_GNA_POWER_CONTROL_SPA;
	while ((regs->gna_power_control & SOC_GNA_POWER_CONTROL_CPA) == 0U) {
		/* wait for power status */
	}

	/* enable clock for GNA block */
	regs->gna_power_control |= SOC_GNA_POWER_CONTROL_CLK_EN;
#endif
}

static inline void soc_set_power_and_clock(void)
{
	volatile struct soc_dsp_shim_regs *dsp_shim_regs =
		(volatile struct soc_dsp_shim_regs *)SOC_DSP_SHIM_REG_BASE;

	dsp_shim_regs->clkctl |= SOC_CLKCTL_REQ_FAST_CLK |
		SOC_CLKCTL_OCS_FAST_CLK;
	dsp_shim_regs->pwrctl |= SOC_PWRCTL_DISABLE_PWR_GATING_DSP0;

	soc_set_dmic_power();
	soc_set_gna_power();
	soc_set_audio_mclk();
}

static inline void soc_read_bootstraps(void)
{
	volatile struct soc_global_regs *regs =
		(volatile struct soc_global_regs *)SOC_S1000_GLB_CTRL_BASE;
	uint32_t bootstrap;

	bootstrap = regs->straps;

	bootstrap &= SOC_S1000_STRAP_REF_CLK;

	switch (bootstrap) {
	case SOC_S1000_STRAP_REF_CLK_19P2:
		ref_clk_freq = 19200000U;
		break;
	case SOC_S1000_STRAP_REF_CLK_24P576:
		ref_clk_freq = 24576000U;
		break;
	case SOC_S1000_STRAP_REF_CLK_38P4:
	default:
		ref_clk_freq = 38400000U;
		break;
	}
}

static int soc_init(const struct device *dev)
{
	soc_read_bootstraps();

	LOG_INF("Reference clock frequency: %u Hz", ref_clk_freq);

	soc_set_resource_ownership();
	soc_set_power_and_clock();

	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 99);
