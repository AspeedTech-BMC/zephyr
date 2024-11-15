if SOC_AST2700_TSP || SOC_AST2700_A0_TSP

config ICACHE_LINE_SIZE
	default 32

config DCACHE_LINE_SIZE
	default 32

choice CACHE_TYPE
	default EXTERNAL_CACHE
endchoice

config XIP
	default y

config LINKER_LAST_SECTION_ID
	default n

config CLOCK_CONTROL_INIT_PRIORITY
	default SYSCON_INIT_PRIORITY

config RESET_INIT_PRIORITY
	default SYSCON_INIT_PRIORITY

config ARM_CUSTOM_INTERRUPT_CONTROLLER
	default y

config MULTI_LEVEL_INTERRUPTS
	default y

config MAX_IRQ_PER_AGGREGATOR
	default 32

config 2ND_LEVEL_INTERRUPTS
	default y

if SOC_AST2700_A0_TSP

config 2ND_LVL_ISR_TBL_OFFSET
	default 160

config NUM_2ND_LEVEL_AGGREGATORS
	default 8

config 2ND_LVL_INTR_00_OFFSET
	default 128

config 2ND_LVL_INTR_01_OFFSET
	default 129

config 2ND_LVL_INTR_02_OFFSET
	default 130

config 2ND_LVL_INTR_03_OFFSET
	default 131

config 2ND_LVL_INTR_04_OFFSET
	default 132

config 2ND_LVL_INTR_05_OFFSET
	default 133

config 2ND_LVL_INTR_06_OFFSET
	default 134

config 2ND_LVL_INTR_07_OFFSET
	default 135

config NUM_IRQS
	default 448

endif # SOC_AST2700_A0_TSP

if SOC_AST2700_TSP

config 2ND_LVL_ISR_TBL_OFFSET
	default 192

config NUM_2ND_LEVEL_AGGREGATORS
	default 1

config 2ND_LVL_INTR_00_OFFSET
	default 160

config 3RD_LEVEL_INTERRUPTS
	default y

config 3RD_LVL_ISR_TBL_OFFSET
	default 224

config NUM_3RD_LEVEL_AGGREGATORS
	default 8

config 3RD_LVL_INTR_00_OFFSET
	default 0

config 3RD_LVL_INTR_01_OFFSET
	default 1

config 3RD_LVL_INTR_02_OFFSET
	default 2

config 3RD_LVL_INTR_03_OFFSET
	default 3

config 3RD_LVL_INTR_04_OFFSET
	default 4

config 3RD_LVL_INTR_05_OFFSET
	default 5

config 3RD_LVL_INTR_06_OFFSET
	default 6

config 3RD_LVL_INTR_07_OFFSET
	default 7

config NUM_IRQS
	default 480

endif # SOC_AST2700_TSP

endif # SOC_AST2700_TSP || SOC_AST2700_A0_TSP