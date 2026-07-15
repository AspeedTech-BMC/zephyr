/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_uart_routing

#include <soc.h>
#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/misc/aspeed/uart_routing_aspeed.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_routing_aspeed);

/* register offsets */
#define HICR9	0x98
#define HICRA	0x9c

/* attributes options */
#define UART_ROUTING_IO0	"io0"
#define UART_ROUTING_IO1	"io1"
#define UART_ROUTING_IO2	"io2"
#define UART_ROUTING_IO3	"io3"
#define UART_ROUTING_IO4	"io4"
#define UART_ROUTING_IO5	"io5"
#define UART_ROUTING_IO6	"io6"
#define UART_ROUTING_IO7	"io7"
#define UART_ROUTING_IO8	"io8"
#define UART_ROUTING_IO9	"io9"
#define UART_ROUTING_IO10	"io10"
#define UART_ROUTING_IO12	"io12"
#define UART_ROUTING_UART0	"uart0"
#define UART_ROUTING_UART1	"uart1"
#define UART_ROUTING_UART2	"uart2"
#define UART_ROUTING_UART3	"uart3"
#define UART_ROUTING_UART4	"uart4"
#define UART_ROUTING_UART5	"uart5"
#define UART_ROUTING_UART6	"uart6"
#define UART_ROUTING_UART7	"uart7"
#define UART_ROUTING_UART8	"uart8"
#define UART_ROUTING_UART9	"uart9"
#define UART_ROUTING_UART10	"uart10"
#define UART_ROUTING_UART12	"uart12"
#define UART_ROUTING_RES	"reserved"

struct aspeed_uart_routing_selector {
	const char *const name;
	uint8_t reg;
	uint8_t mask;
	uint8_t shift;
	const char *const options[];
};

/* routing selector for AST26xx */
static struct aspeed_uart_routing_selector ast2600_uart10_sel = {
	.name = UART_ROUTING_UART10,
	.reg = HICR9,
	.shift = 12,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_IO10,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
			UART_ROUTING_RES,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_uart4_sel = {
	.name = UART_ROUTING_UART4,
	.reg = HICRA,
	.shift = 25,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO4,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_IO10,
		    NULL,
	},
};

static struct aspeed_uart_routing_selector ast2600_uart3_sel = {
	.name = UART_ROUTING_UART3,
	.reg = HICRA,
	.shift = 22,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_uart2_sel = {
	.name = UART_ROUTING_UART2,
	.reg = HICRA,
	.shift = 19,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
		    UART_ROUTING_IO1,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_uart1_sel = {
	.name = UART_ROUTING_UART1,
	.reg = HICRA,
	.shift = 16,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_io10_sel = {
	.name = UART_ROUTING_IO10,
	.reg = HICR9,
	.shift = 8,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
			UART_ROUTING_RES,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
			UART_ROUTING_RES,
		    UART_ROUTING_UART10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_io4_sel = {
	.name = UART_ROUTING_IO4,
	.reg = HICRA,
	.shift = 9,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_io3_sel = {
	.name = UART_ROUTING_IO3,
	.reg = HICRA,
	.shift = 6,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_io2_sel = {
	.name = UART_ROUTING_IO2,
	.reg = HICRA,
	.shift = 3,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2600_io1_sel = {
	.name = UART_ROUTING_IO1,
	.reg = HICRA,
	.shift = 0,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART4,
		    UART_ROUTING_UART10,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO4,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_uart9_sel = {
	.name = UART_ROUTING_UART9,
	.reg = HICR9,
	.shift = 12,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_IO9,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
			UART_ROUTING_RES,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART12,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_uart3_sel = {
	.name = UART_ROUTING_UART3,
	.reg = HICRA,
	.shift = 25,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO9,
		    NULL,
	},
};

static struct aspeed_uart_routing_selector ast2700n0_uart2_sel = {
	.name = UART_ROUTING_UART2,
	.reg = HICRA,
	.shift = 22,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_uart1_sel = {
	.name = UART_ROUTING_UART1,
	.reg = HICRA,
	.shift = 19,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART0,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_uart0_sel = {
	.name = UART_ROUTING_UART0,
	.reg = HICRA,
	.shift = 16,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_io9_sel = {
	.name = UART_ROUTING_IO9,
	.reg = HICR9,
	.shift = 8,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART12,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
			UART_ROUTING_RES,
		    UART_ROUTING_UART9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_io3_sel = {
	.name = UART_ROUTING_IO3,
	.reg = HICRA,
	.shift = 9,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_io2_sel = {
	.name = UART_ROUTING_IO2,
	.reg = HICRA,
	.shift = 6,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_io1_sel = {
	.name = UART_ROUTING_IO1,
	.reg = HICRA,
	.shift = 3,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n0_io0_sel = {
	.name = UART_ROUTING_IO0,
	.reg = HICRA,
	.shift = 0,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

/* routing selector for AST27xx node 1 */
static struct aspeed_uart_routing_selector ast2700n1_uart10_sel = {
	.name = UART_ROUTING_UART10,
	.reg = HICR9,
	.shift = 12,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_IO10,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
			UART_ROUTING_RES,
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART12,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_uart8_sel = {
	.name = UART_ROUTING_UART8,
	.reg = HICRA,
	.shift = 25,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO8,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO7,
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_IO10,
		    NULL,
	},
};

static struct aspeed_uart_routing_selector ast2700n1_uart7_sel = {
	.name = UART_ROUTING_UART7,
	.reg = HICRA,
	.shift = 22,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_uart6_sel = {
	.name = UART_ROUTING_UART6,
	.reg = HICRA,
	.shift = 19,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
		    UART_ROUTING_IO5,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART5,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_uart5_sel = {
	.name = UART_ROUTING_UART5,
	.reg = HICRA,
	.shift = 16,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_io10_sel = {
	.name = UART_ROUTING_IO10,
	.reg = HICR9,
	.shift = 8,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART12,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
			UART_ROUTING_RES,
		    UART_ROUTING_UART10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_io8_sel = {
	.name = UART_ROUTING_IO8,
	.reg = HICRA,
	.shift = 9,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_io7_sel = {
	.name = UART_ROUTING_IO7,
	.reg = HICRA,
	.shift = 6,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_IO5,
		    UART_ROUTING_IO6,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_io6_sel = {
	.name = UART_ROUTING_IO6,
	.reg = HICRA,
	.shift = 3,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART10,
		    UART_ROUTING_UART5,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast2700n1_io5_sel = {
	.name = UART_ROUTING_IO5,
	.reg = HICRA,
	.shift = 0,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART5,
		    UART_ROUTING_UART6,
		    UART_ROUTING_UART7,
		    UART_ROUTING_UART8,
		    UART_ROUTING_UART10,
		    UART_ROUTING_IO7,
		    UART_ROUTING_IO8,
		    UART_ROUTING_IO10,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_uart9_sel = {
	.name = UART_ROUTING_UART9,
	.reg = HICR9,
	.shift = 12,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_IO9,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
			UART_ROUTING_RES,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART12,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_uart3_sel = {
	.name = UART_ROUTING_UART3,
	.reg = HICRA,
	.shift = 25,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO9,
		    NULL,
	},
};

static struct aspeed_uart_routing_selector ast1040_uart2_sel = {
	.name = UART_ROUTING_UART2,
	.reg = HICRA,
	.shift = 22,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_uart1_sel = {
	.name = UART_ROUTING_UART1,
	.reg = HICRA,
	.shift = 19,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO0,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART0,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_uart0_sel = {
	.name = UART_ROUTING_UART0,
	.reg = HICRA,
	.shift = 16,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_io9_sel = {
	.name = UART_ROUTING_IO9,
	.reg = HICR9,
	.shift = 8,
	.mask = 0xf,
	.options = {
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART12,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_RES,
		    UART_ROUTING_UART9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_io3_sel = {
	.name = UART_ROUTING_IO3,
	.reg = HICRA,
	.shift = 9,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_io2_sel = {
	.name = UART_ROUTING_IO2,
	.reg = HICRA,
	.shift = 6,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_IO0,
		    UART_ROUTING_IO1,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_io1_sel = {
	.name = UART_ROUTING_IO1,
	.reg = HICRA,
	.shift = 3,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_UART0,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector ast1040_io0_sel = {
	.name = UART_ROUTING_IO0,
	.reg = HICRA,
	.shift = 0,
	.mask = 0x7,
	.options = {
		    UART_ROUTING_UART0,
		    UART_ROUTING_UART1,
		    UART_ROUTING_UART2,
		    UART_ROUTING_UART3,
		    UART_ROUTING_UART9,
		    UART_ROUTING_IO2,
		    UART_ROUTING_IO3,
		    UART_ROUTING_IO9,
		    NULL,
		    },
};

static struct aspeed_uart_routing_selector *const ast2600_uart_routing_attrs[] = {
	&ast2600_uart10_sel,
	&ast2600_uart4_sel,
	&ast2600_uart3_sel,
	&ast2600_uart2_sel,
	&ast2600_uart1_sel,
	&ast2600_io10_sel,
	&ast2600_io4_sel,
	&ast2600_io3_sel,
	&ast2600_io2_sel,
	&ast2600_io1_sel,
	NULL,
};

static struct aspeed_uart_routing_selector *const ast2700n0_uart_routing_attrs[] = {
	&ast2700n0_uart9_sel,
	&ast2700n0_uart3_sel,
	&ast2700n0_uart2_sel,
	&ast2700n0_uart1_sel,
	&ast2700n0_uart0_sel,
	&ast2700n0_io9_sel,
	&ast2700n0_io3_sel,
	&ast2700n0_io2_sel,
	&ast2700n0_io1_sel,
	&ast2700n0_io0_sel,
	NULL,
};

static struct aspeed_uart_routing_selector *const ast2700n1_uart_routing_attrs[] = {
	&ast2700n1_uart10_sel,
	&ast2700n1_uart8_sel,
	&ast2700n1_uart7_sel,
	&ast2700n1_uart6_sel,
	&ast2700n1_uart5_sel,
	&ast2700n1_io10_sel,
	&ast2700n1_io8_sel,
	&ast2700n1_io7_sel,
	&ast2700n1_io6_sel,
	&ast2700n1_io5_sel,
	NULL,
};

static struct aspeed_uart_routing_selector *const ast1040_uart_routing_attrs[] = {
	&ast1040_uart9_sel,
	&ast1040_uart3_sel,
	&ast1040_uart2_sel,
	&ast1040_uart1_sel,
	&ast1040_uart0_sel,
	&ast1040_io9_sel,
	&ast1040_io3_sel,
	&ast1040_io2_sel,
	&ast1040_io1_sel,
	&ast1040_io0_sel,
	NULL,
};

struct uart_routing_chip_data {
	const char *compatible;
	struct aspeed_uart_routing_selector *const *attrs;
};

static const struct uart_routing_chip_data uart_routing_chip_table[] = {
	{ .compatible = "aspeed,ast2600-uart-routing", .attrs = ast2600_uart_routing_attrs },
	{ .compatible = "aspeed,ast2700n0-uart-routing", .attrs = ast2700n0_uart_routing_attrs },
	{ .compatible = "aspeed,ast2700n1-uart-routing", .attrs = ast2700n1_uart_routing_attrs },
	{ .compatible = "aspeed,ast1040-uart-routing", .attrs = ast1040_uart_routing_attrs },
};

static struct aspeed_uart_routing_selector *const *
uart_routing_aspeed_match_data(const char *compatible)
{
	for (size_t i = 0; i < ARRAY_SIZE(uart_routing_chip_table); i++) {
		if (strcmp(uart_routing_chip_table[i].compatible, compatible) == 0) {
			return uart_routing_chip_table[i].attrs;
		}
	}

	return NULL;
}

struct uart_routing_aspeed_config {
	uintptr_t base;
};

struct uart_routing_aspeed_data {
	struct aspeed_uart_routing_selector *const *attrs;
};

#define LSC_RD(cfg, reg)		sys_read32((cfg)->base + (reg))
#define LSC_WR(cfg, val, reg)   sys_write32((val), (cfg)->base + (reg))

int uart_routing_aspeed_set(const struct device *dev, const char *name, const char *target)
{
	const struct uart_routing_aspeed_config *cfg = dev->config;
	struct uart_routing_aspeed_data *data = dev->data;
	struct aspeed_uart_routing_selector *sel;
	uint32_t reg;
	int idx;

	for (int i = 0; data->attrs[i]; i++) {
		sel = data->attrs[i];

		if (strcmp(sel->name, name) != 0) {
			continue;
		}

		idx = -1;
		for (int j = 0; sel->options[j]; j++) {
			if (strcmp(sel->options[j], target) == 0) {
				idx = j;
				break;
			}
		}

		if (idx < 0) {
			LOG_ERR("\"%s\" cannot be routed to \"%s\"", name, target);
			return -EINVAL;
		}

		reg = LSC_RD(cfg, sel->reg);
		reg &= ~(sel->mask << sel->shift);
		reg |= (idx & sel->mask) << sel->shift;
		LSC_WR(cfg, reg, sel->reg);
		return 0;
	}

	LOG_ERR("unknown uart-routing channel \"%s\"", name);

	return -EINVAL;
}

int uart_routing_aspeed_get(const struct device *dev, const char *name, const char **target)
{
	const struct uart_routing_aspeed_config *cfg = dev->config;
	struct uart_routing_aspeed_data *data = dev->data;
	struct aspeed_uart_routing_selector *sel;
	uint32_t reg;
	int idx;

	for (int i = 0; data->attrs[i]; i++) {
		sel = data->attrs[i];

		if (strcmp(sel->name, name) != 0) {
			continue;
		}

		reg = LSC_RD(cfg, sel->reg);
		idx = (reg >> sel->shift) & sel->mask;

		for (int j = 0; sel->options[j]; j++) {
			if (j == idx) {
				*target = sel->options[j];
				return 0;
			}
		}

		LOG_ERR("\"%s\" has an unrecognized routing value (%d)", name, idx);
		return -ERANGE;
	}

	LOG_ERR("unknown uart-routing channel \"%s\"", name);

	return -EINVAL;
}

static int uart_routing_aspeed_init(const struct device *dev)
{
	struct uart_routing_aspeed_data *data = dev->data;

	data->attrs = uart_routing_aspeed_match_data(DT_INST_PROP_BY_IDX(0, compatible, 0));
	if (!data->attrs) {
		LOG_ERR("no uart-routing table for compatible \"%s\"",
			DT_INST_PROP_BY_IDX(0, compatible, 0));
		return -ENODEV;
	}

#define UART_ROUTING_APPLY_CH(node_id, prop, idx)				\
	uart_routing_aspeed_set(dev, DT_PROP_BY_IDX(node_id, ch_name, idx),	\
				 DT_PROP_BY_IDX(node_id, prop, idx));

#define UART_ROUTING_APPLY_IO(node_id, prop, idx)				\
	uart_routing_aspeed_set(dev, DT_PROP_BY_IDX(node_id, io_name, idx),	\
				 DT_PROP_BY_IDX(node_id, prop, idx));

	DT_INST_FOREACH_PROP_ELEM(0, sel_io, UART_ROUTING_APPLY_CH);
	DT_INST_FOREACH_PROP_ELEM(0, sel_ch, UART_ROUTING_APPLY_IO);

#undef UART_ROUTING_APPLY_CH
#undef UART_ROUTING_APPLY_IO

	return 0;
}

static const struct uart_routing_aspeed_config uart_routing_aspeed_config = {
	.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(0))),
};

static struct uart_routing_aspeed_data uart_routing_aspeed_data;

DEVICE_DT_INST_DEFINE(0, uart_routing_aspeed_init, NULL,
		      &uart_routing_aspeed_data, &uart_routing_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
