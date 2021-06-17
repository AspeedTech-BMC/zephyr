/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_pcc

#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/npcx_clock.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control_npcx, LOG_LEVEL_ERR);

/* Driver config */
struct npcx_pcc_config {
	/* cdcg device base address */
	uintptr_t base_cdcg;
	/* pmc device base address */
	uintptr_t base_pmc;
};

/* Driver convenience defines */
#define DRV_CONFIG(dev) \
	((const struct npcx_pcc_config *)(dev)->config)

#define HAL_CDCG_INST(dev) \
	(struct cdcg_reg *)(DRV_CONFIG(dev)->base_cdcg)

#define HAL_PMC_INST(dev) \
	(struct pmc_reg *)(DRV_CONFIG(dev)->base_pmc)

/* Clock controller local functions */
static inline int npcx_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct npcx_clk_cfg *clk_cfg = (struct npcx_clk_cfg *)(sub_system);
	const uint32_t pmc_base = DRV_CONFIG(dev)->base_pmc;

	if (clk_cfg->ctrl >= NPCX_PWDWN_CTL_COUNT)
		return -EINVAL;

	/* Clear related PD (Power-Down) bit of module to turn on clock */
	NPCX_PWDWN_CTL(pmc_base, clk_cfg->ctrl) &= ~(BIT(clk_cfg->bit));
	return 0;
}

static inline int npcx_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct npcx_clk_cfg *clk_cfg = (struct npcx_clk_cfg *)(sub_system);
	const uint32_t pmc_base = DRV_CONFIG(dev)->base_pmc;

	if (clk_cfg->ctrl >= NPCX_PWDWN_CTL_COUNT)
		return -EINVAL;

	/* Set related PD (Power-Down) bit of module to turn off clock */
	NPCX_PWDWN_CTL(pmc_base, clk_cfg->ctrl) |= BIT(clk_cfg->bit);
	return 0;
}

static int npcx_clock_control_get_subsys_rate(const struct device *dev,
					      clock_control_subsys_t sub_system,
					      uint32_t *rate)
{
	ARG_UNUSED(dev);
	struct npcx_clk_cfg *clk_cfg = (struct npcx_clk_cfg *)(sub_system);

	switch (clk_cfg->bus) {
	case NPCX_CLOCK_BUS_APB1:
		*rate = NPCX_APB_CLOCK(1);
		break;
	case NPCX_CLOCK_BUS_APB2:
		*rate = NPCX_APB_CLOCK(2);
		break;
	case NPCX_CLOCK_BUS_APB3:
		*rate = NPCX_APB_CLOCK(3);
		break;
	case NPCX_CLOCK_BUS_AHB6:
		*rate = CORE_CLK/(AHB6DIV_VAL + 1);
		break;
	case NPCX_CLOCK_BUS_FIU:
		*rate = CORE_CLK/(FIUDIV_VAL + 1);
		break;
	case NPCX_CLOCK_BUS_CORE:
		*rate = CORE_CLK;
		break;
	case NPCX_CLOCK_BUS_LFCLK:
		*rate = LFCLK;
		break;
	default:
		*rate = 0U;
		/* Invalid parameters */
		return -EINVAL;
	}

	return 0;
}

/* Platform specific clock controller functions */
#if defined(CONFIG_PM)
void npcx_clock_control_turn_on_system_sleep(bool is_deep, bool is_instant)
{
	const struct device *const clk_dev =
					device_get_binding(NPCX_CLK_CTRL_NAME);
	struct pmc_reg *const inst_pmc = HAL_PMC_INST(clk_dev);
	/* Configure that ec enters system sleep mode if receiving 'wfi' */
	uint8_t pm_flags = BIT(NPCX_PMCSR_IDLE);

	/* Add 'Disable High-Frequency' flag (ie. 'deep sleep' mode) */
	if (is_deep) {
		pm_flags |= BIT(NPCX_PMCSR_DHF);
		/* Add 'Instant Wake-up' flag if sleep time is within 200 ms */
		if (is_instant)
			pm_flags |= BIT(NPCX_PMCSR_DI_INSTW);
	}

	inst_pmc->PMCSR = pm_flags;
}

void npcx_clock_control_turn_off_system_sleep(void)
{
	const struct device *const clk_dev =
					device_get_binding(NPCX_CLK_CTRL_NAME);
	struct pmc_reg *const inst_pmc = HAL_PMC_INST(clk_dev);

	inst_pmc->PMCSR = 0;
}
#endif /* CONFIG_PM */

/* Clock controller driver registration */
static struct clock_control_driver_api npcx_clock_control_api = {
	.on = npcx_clock_control_on,
	.off = npcx_clock_control_off,
	.get_rate = npcx_clock_control_get_subsys_rate,
};

/* valid clock frequency check */
BUILD_ASSERT(CORE_CLK <= 100000000 &&
	     CORE_CLK >= 4000000 &&
	     OSC_CLK % CORE_CLK == 0 &&
	     OSC_CLK / CORE_CLK <= 10,
	     "Invalid CORE_CLK setting");
BUILD_ASSERT(CORE_CLK / (FIUDIV_VAL + 1) <= 50000000 &&
	     CORE_CLK / (FIUDIV_VAL + 1) >= 4000000,
	     "Invalid FIUCLK setting");
BUILD_ASSERT(CORE_CLK / (AHB6DIV_VAL + 1) <= 50000000 &&
	     CORE_CLK / (AHB6DIV_VAL + 1) >= 4000000,
	     "Invalid AHB6_CLK setting");
BUILD_ASSERT(APBSRC_CLK / (APB1DIV_VAL + 1) <= 50000000 &&
	     APBSRC_CLK / (APB1DIV_VAL + 1) >= 4000000 &&
	     (APB1DIV_VAL + 1) % (FPRED_VAL + 1) == 0,
	     "Invalid APB1_CLK setting");
BUILD_ASSERT(APBSRC_CLK / (APB2DIV_VAL + 1) <= 50000000 &&
	     APBSRC_CLK / (APB2DIV_VAL + 1) >= 8000000 &&
	     (APB2DIV_VAL + 1) % (FPRED_VAL + 1) == 0,
	     "Invalid APB2_CLK setting");
BUILD_ASSERT(APBSRC_CLK / (APB3DIV_VAL + 1) <= 50000000 &&
	     APBSRC_CLK / (APB3DIV_VAL + 1) >= 12500000 &&
	     (APB3DIV_VAL + 1) % (FPRED_VAL + 1) == 0,
	     "Invalid APB3_CLK setting");

static int npcx_clock_control_init(const struct device *dev)
{
	struct cdcg_reg *const inst_cdcg = HAL_CDCG_INST(dev);
	const uint32_t pmc_base = DRV_CONFIG(dev)->base_pmc;

	/*
	 * Resetting the OSC_CLK (even to the same value) will make the clock
	 * unstable for a little which can affect peripheral communication like
	 * eSPI. Skip this if not needed.
	 */
	if (inst_cdcg->HFCGN != HFCGN_VAL || inst_cdcg->HFCGML != HFCGML_VAL
			|| inst_cdcg->HFCGMH != HFCGMH_VAL) {
		/*
		 * Configure frequency multiplier M/N values according to
		 * the requested OSC_CLK (Unit:Hz).
		 */
		inst_cdcg->HFCGN  = HFCGN_VAL;
		inst_cdcg->HFCGML = HFCGML_VAL;
		inst_cdcg->HFCGMH = HFCGMH_VAL;

		/* Load M and N values into the frequency multiplier */
		inst_cdcg->HFCGCTRL |= BIT(NPCX_HFCGCTRL_LOAD);
		/* Wait for stable */
		while (IS_BIT_SET(inst_cdcg->HFCGCTRL, NPCX_HFCGCTRL_CLK_CHNG))
			;
	}

	/* Set all clock prescalers of core and peripherals. */
	inst_cdcg->HFCGP   = ((FPRED_VAL << 4) | AHB6DIV_VAL);
	inst_cdcg->HFCBCD  = (FIUDIV_VAL << 4);
	inst_cdcg->HFCBCD1 = (APB1DIV_VAL | (APB2DIV_VAL << 4));
	inst_cdcg->HFCBCD2 = APB3DIV_VAL;

	/*
	 * Power-down (turn off clock) the modules initially for better
	 * power consumption.
	 */
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL1) = 0xF9; /* No SDP_PD/FIU_PD */
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL2) = 0xFF;
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL3) = 0x1F; /* No GDMA_PD */
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL4) = 0xFF;
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL5) = 0xFA;
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL6) = 0xFF;
	NPCX_PWDWN_CTL(pmc_base, NPCX_PWDWN_CTL7) = 0xE7;

	return 0;
}

const struct npcx_pcc_config pcc_config = {
	.base_cdcg = DT_INST_REG_ADDR_BY_NAME(0, cdcg),
	.base_pmc  = DT_INST_REG_ADDR_BY_NAME(0, pmc),
};

DEVICE_DT_INST_DEFINE(0,
		    &npcx_clock_control_init,
		    NULL,
		    NULL, &pcc_config,
		    PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_OBJECTS,
		    &npcx_clock_control_api);
