/*
 * Copyright (c) 2020 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_PM_STATE_H_
#define ZEPHYR_INCLUDE_PM_STATE_H_

#include <sys/util.h>
#include <devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup pm_states Power Management States
 * @ingroup power_management_api
 * @{
 */

/**
 * @enum pm_state Power management state
 */
enum pm_state {
	/**
	 * @brief Runtime active state
	 *
	 * The system is fully powered and active.
	 *
	 * @note This state is correlated with ACPI G0/S0 state
	 */
	PM_STATE_ACTIVE,
	/**
	 * @brief Runtime idle state
	 *
	 * Runtime idle is a system sleep state in which all of the cores
	 * enter deepest possible idle state and wait for interrupts, no
	 * requirements for the devices, leaving them at the states where
	 * they are.
	 *
	 * @note This state is correlated with ACPI S0ix state
	 */
	PM_STATE_RUNTIME_IDLE,
	/**
	 * @brief Suspend to idle state
	 *
	 * The system goes through a normal platform suspend where it puts
	 * all of the cores in deepest possible idle state and *may* puts peripherals
	 * into low-power states. No operating state is lost (ie. the cpu core
	 * does not lose execution context), so the system can go back to where
	 * it left off easily enough.
	 *
	 * @note This state is correlated with ACPI S1 state
	 */
	PM_STATE_SUSPEND_TO_IDLE,
	/**
	 * @brief Standby state
	 *
	 * In addition to putting peripherals into low-power states all
	 * non-boot CPUs are powered off. It should allow more energy to be
	 * saved relative to suspend to idle, but the resume latency will
	 * generally be greater than for that state. But it should be the same
	 * state with suspend to idle state on uniprocesser system.
	 *
	 * @note This state is correlated with ACPI S2 state
	 */
	PM_STATE_STANDBY,
	/**
	 * @brief Suspend to ram state
	 *
	 * This state offers significant energy savings by powering off as much
	 * of the system as possible, where memory should be placed into the
	 * self-refresh mode to retain its contents. The state of devices and
	 * CPUs is saved and held in memory, and it may require some boot-
	 * strapping code in ROM to resume the system from it.
	 *
	 * @note This state is correlated with ACPI S3 state
	 */
	PM_STATE_SUSPEND_TO_RAM,
	/**
	 * @brief Suspend to disk state
	 *
	 * This state offers significant energy savings by powering off as much
	 * of the system as possible, including the memory. The contents of
	 * memory are written to disk or other non-volatile storage, and on resume
	 * it's read back into memory with the help of boot-strapping code,
	 * restores the system to the same point of execution where it went to
	 * suspend to disk.
	 *
	 * @note This state is correlated with ACPI S4 state
	 */
	PM_STATE_SUSPEND_TO_DISK,
	/**
	 * @brief Soft off state
	 *
	 * This state consumes a minimal amount of power and requires a large
	 * latency in order to return to runtime active state. The contents of
	 * system(CPU and memory) will not be preserved, so the system will be
	 * restarted as if from initial power-up and kernel boot.
	 *
	 * @note This state is correlated with ACPI G2/S5 state
	 */
	PM_STATE_SOFT_OFF
};

/**
 * Information about a power management state
 */
struct pm_state_info {
	enum pm_state state;

	/**
	 * Some platforms have multiple states that map to
	 * one Zephyr power state. This property allows the platform
	 * distinguish them. e.g:
	 *
	 *	power-states {
	 *		state0: state0 {
	 *			compatible = "zephyr,power-state";
	 *			power-state-name = "suspend-to-idle";
	 *			substate-id = <1>;
	 *			min-residency-us = <10000>;
	 *			exit-latency-us = <100>;
	 *		};
	 *		state1: state1 {
	 *			compatible = "zephyr,power-state";
	 *			power-state-name = "suspend-to-idle";
	 *			substate-id = <2>;
	 *			min-residency-us = <20000>;
	 *			exit-latency-us = <200>;
	 *		};
	 *	}
	 */
	uint8_t substate_id;

	/**
	 * Minimum residency duration in microseconds. It is the minimum
	 * time for a given idle state to be worthwhile energywise.
	 *
	 * @note 0 means that this property is not available for this state.
	 */
	uint32_t min_residency_us;

	/**
	 * Worst case latency in microseconds required to exit the idle state.
	 *
	 * @note 0 means that this property is not available for this state.
	 */
	uint32_t exit_latency_us;
};

/**
 * @brief Construct a pm_state_info from 'cpu-power-states' property at index 'i'
 *
 * @param node_id A node identifier with compatible zephyr,power-state
 * @param i index into cpu-power-states property
 * @return pm_state_info item from 'cpu-power-states' property at index 'i'
 */
#define PM_STATE_INFO_DT_ITEM_BY_IDX(node_id, i)                    \
	{                                                           \
		.state = DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node_id,	\
			cpu_power_states, i), power_state_name),    \
		.substate_id = DT_PROP_BY_PHANDLE_IDX_OR(node_id, \
			cpu_power_states, i, substate_id, 0),   \
		.min_residency_us = DT_PROP_BY_PHANDLE_IDX_OR(node_id, \
				cpu_power_states, i, min_residency_us, 0),\
		.exit_latency_us = DT_PROP_BY_PHANDLE_IDX_OR(node_id, \
				cpu_power_states, i, exit_latency_us, 0),\
	},

/**
 * @brief Length of 'cpu-power-states' property
 *
 * @param node_id A node identifier with compatible zephyr,power-state
 * @return length of 'cpu-power-states' property
 */
#define PM_STATE_DT_ITEMS_LEN(node_id) \
	DT_PROP_LEN_OR(node_id, cpu_power_states, 0)

/**
 * @brief Macro function to construct enum pm_state item in UTIL_LISTIFY
 * extension.
 *
 * @param child child index in UTIL_LISTIFY extension.
 * @param node_id A node identifier with compatible zephyr,power-state
 * @return macro function to construct a pm_state_info
 */
#define PM_STATE_INFO_DT_ITEMS_LISTIFY_FUNC(child, node_id) \
	PM_STATE_INFO_DT_ITEM_BY_IDX(node_id, child)

/**
 * @brief Macro function to construct a list of 'pm_state_info' items by
 * UTIL_LISTIFY func
 *
 * Example devicetree fragment:
 *	cpus {
 *		...
 *		cpu0: cpu@0 {
 *			device_type = "cpu";
 *			...
 *			cpu-power-states = <&state0 &state1>;
 *		};
 *	};
 *
 *	...
 *      power-states {
 *		state0: state0 {
 *			compatible = "zephyr,power-state";
 *			power-state-name = "suspend-to-idle";
 *			min-residency-us = <10000>;
 *		        exit-latency-us = <100>;
 *		};
 *
 *		state1: state1 {
 *			compatible = "zephyr,power-state";
 *			power-state-name = "suspend-to-ram";
 *			min-residency-us = <50000>;
 *		        exit-latency-us = <500>;
 *		};
 *	};
 *
 * Example usage: *
 *    const enum pm_state states[] =
 *		PM_STATE_DT_INFO_ITEMS_LIST(DT_NODELABEL(cpu0));
 *
 * @param node_id A node identifier with compatible zephyr,power-state
 * @return an array of struct pm_state_info.
 */
#define PM_STATE_INFO_DT_ITEMS_LIST(node_id) {         \
	UTIL_LISTIFY(PM_STATE_DT_ITEMS_LEN(node_id),   \
		     PM_STATE_INFO_DT_ITEMS_LISTIFY_FUNC,\
		     node_id)                          \
	}

/**
 * @brief Construct a pm_state enum from 'cpu-power-states' property
 *        at index 'i'
 *
 * @param node_id A node identifier with compatible zephyr,power-state
 * @param i index into cpu-power-states property
 * @return pm_state item from 'cpu-power-states' property at index 'i'
 */
#define PM_STATE_DT_ITEM_BY_IDX(node_id, i)			\
	DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node_id,			\
		      cpu_power_states, i), power_state_name),


/**
 * @brief Macro function to construct enum pm_state item in UTIL_LISTIFY
 * extension.
 *
 * @param child child index in UTIL_LISTIFY extension.
 * @param node_id A node identifier with compatible zephyr,power-state
 * @return macro function to construct a pm_state enum
 */
#define PM_STATE_DT_ITEMS_LISTIFY_FUNC(child, node_id) \
	PM_STATE_DT_ITEM_BY_IDX(node_id, child)

/**
 * @brief Macro function to construct a list of enum pm_state items by
 * UTIL_LISTIFY func
 *
 * Example devicetree fragment:
 *	cpus {
 *		...
 *		cpu0: cpu@0 {
 *			device_type = "cpu";
 *			...
 *			cpu-power-states = <&state0 &state1>;
 *		};
 *	};
 *
 *	...
 *	state0: state0 {
 *		compatible = "zephyr,power-state";
 *		power-state-name = "suspend-to-idle";
 *		min-residency-us = <10000>;
 *		exit-latency-us = <100>;
 *	};
 *
 *	state1: state1 {
 *		compatible = "zephyr,power-state";
 *		power-state-name = "suspend-to-ram";
 *		min-residency-us = <50000>;
 *		exit-latency-us = <500>;
 *	};
 *
 * Example usage: *
 *    const enum pm_state states[] = PM_STATE_DT_ITEMS_LIST(DT_NODELABEL(cpu0));
 *
 * @param node_id A node identifier with compatible zephyr,power-state
 * @return an array of enum pm_state items.
 */
#define PM_STATE_DT_ITEMS_LIST(node_id) {           \
	UTIL_LISTIFY(PM_STATE_DT_ITEMS_LEN(node_id),\
		     PM_STATE_DT_ITEMS_LISTIFY_FUNC,\
		     node_id)                       \
	}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
