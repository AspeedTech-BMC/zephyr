/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _TACH_ASPEED_H_
#define _TACH_ASPEED_H_
/**********************************************************
 * Tach register fields
 *********************************************************/
union tach_general_register_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t tach_threshold : 20;                  /*[0-19]*/
		volatile uint32_t tach_clock_division : 4;              /*[20-23]*/
		volatile uint32_t tach_edge : 2;                        /*[24-25]*/
		volatile uint32_t tach_debounce : 2;                    /*[26-27]*/
		volatile uint32_t enable_tach : 1;                      /*[28-28]*/
		volatile uint32_t enable_tach_loopback_mode : 1;        /*[29-29]*/
		volatile uint32_t inverse_tach_limit_comparison : 1;    /*[30-30]*/
		volatile uint32_t enable_tach_interrupt : 1;            /*[31-31]*/

	} fields;
}; /* 00000008 */

union tach_status_register_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t tach_value : 20;                              /*[0-19]*/
		volatile uint32_t tach_full_measurement : 1;                    /*[20-20]*/
		volatile uint32_t tach_value_updated_since_last_read : 1;       /*[21-21]*/
		volatile uint32_t tach_raw_input : 1;                           /*[22-22]*/
		volatile uint32_t tach_debounce_input : 1;                      /*[23-23]*/
		volatile uint32_t pwm_out_status : 1;                           /*[24-24]*/
		volatile uint32_t pwm_oen_status : 1;                           /*[25-25]*/
		volatile uint32_t reserved: 5;                                  /*[26-30]*/
		volatile uint32_t interrupt_status_and_clear : 1;               /*[31-31]*/

	} fields;
}; /* 0000000c */

struct tach_register_s {
	uint32_t reserved[2];
	union tach_general_register_s tach_general;
	union tach_status_register_s tach_status;
};

#endif /* end of "#ifndef _TACH_ASPEED_H_" */
