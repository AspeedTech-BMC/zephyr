/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

#include <logging/debug_log.h>

#include "engineManager/engine_manager.h"
#include "include/definitions.h"

//struct engine_instances engineInstances;

static int tektagonPlatformInit()
{
	int status = 0;

	printk("Inside tektagon_platform_init\r\n");
	status = initializeEngines(/*&engineInstances*/);
	if(status)
		return status;
	printk("After initializeEngines\r\n");

	status = initializeManifestProcessor();
	if(status)
			return status;
	printk("After InitializeManifestProcessor\r\n");

	return status;
}

void main(void)
{
	int status = 0;

	DEBUG_PRINTF("\r\n *** Tektagon OE version 1.0 ***\r\n");

	// clear internal logging
	status = debug_log_clear();
	if(status)
		return status;

	status = tektagonPlatformInit();
	if(status)
			return status;
	printk("After tektagon_platform_init()\r\n");

	// Process PFM from Flash into Cerberus library
	processPfmFlashManifest();
	printk("After processPfmFlashManifest()\r\n");
}
