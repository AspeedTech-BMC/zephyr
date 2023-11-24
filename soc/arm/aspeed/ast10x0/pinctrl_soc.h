#ifndef ZEPHYR_SOC_ASPEED_AST10X0_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ASPEED_AST10X0_PINCTRL_SOC_H_

#include <zephyr/types.h>

#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.sig_descs = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs),            \
		.num_of_descs = DT_PROP_LEN(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs),     \
		.ball = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), ball),                      \
		.name = DT_NODE_FULL_NAME(DT_PHANDLE_BY_IDX(node_id, prop, idx)),                  \
	},
#else
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.sig_descs = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs),            \
		.num_of_descs = DT_PROP_LEN(DT_PHANDLE_BY_IDX(node_id, prop, idx), sig_descs),     \
		.ball = DT_PROP(DT_PHANDLE_BY_IDX(node_id, prop, idx), ball),                      \
	},
#endif

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)                      \
	}
typedef struct pinctrl_soc_pin {
	uint32_t pincfg;
	int ball;
#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	char *name;
#endif
	int num_of_descs;
	uint32_t sig_descs[3];
} pinctrl_soc_pin_t;
#endif
