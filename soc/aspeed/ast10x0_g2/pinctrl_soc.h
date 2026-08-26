#ifndef ZEPHYR_SOC_ASPEED_AST10X0_G2_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ASPEED_AST10X0_G2_PINCTRL_SOC_H_

#include <zephyr/types.h>

#define Z_PINCTRL_STATE_PIN_PINCFG_INIT(node_id, prop, idx)                                        \
	.bias_disable_desc = DT_PROP_OR(DT_PHANDLE_BY_IDX(node_id, prop, idx),                     \
					 bias_disable_desc, 0),                                    \
	.bias_disable = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), bias_disable),              \
	.drive_strength_desc = DT_PROP_OR(DT_PHANDLE_BY_IDX(node_id, prop, idx),                   \
					   drive_strength_desc, 0),                                \
	.drive_strength = DT_PROP_OR(DT_PHANDLE_BY_IDX(node_id, prop, idx), drive_strength, 0),    \
	.drive_strength_valid = DT_NODE_HAS_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx),            \
						  drive_strength),

#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.sig_descs = DT_PROP_OR(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs, 0),      \
		.ball = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), ball),                      \
		Z_PINCTRL_STATE_PIN_PINCFG_INIT(node_id, prop, idx)                                \
		.name = DT_NODE_FULL_NAME(DT_PHANDLE_BY_IDX(node_id, prop, idx)),                  \
	},
#else
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.sig_descs = DT_PROP_OR(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs, 0),      \
		.ball = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), ball),                      \
		Z_PINCTRL_STATE_PIN_PINCFG_INIT(node_id, prop, idx)                                \
	},
#endif

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)                      \
	}
typedef struct pinctrl_soc_pin {
	int ball;
#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	char *name;
#endif
	uint32_t sig_descs;
	/* Set via bias-disable-desc / bias-disable in the pin's devicetree node. */
	uint32_t bias_disable_desc;
	bool bias_disable;
	/* Set via drive-strength-desc / drive-strength in the pin's devicetree node. */
	uint32_t drive_strength_desc;
	uint32_t drive_strength;
	bool drive_strength_valid;
} pinctrl_soc_pin_t;
#endif
