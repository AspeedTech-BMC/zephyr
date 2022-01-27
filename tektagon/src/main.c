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
#include "platform.h"
#include <kernel.h>
#include <flash/flash_wrapper.h>
#include <crypto/hash_wrapper.h>
//struct engine_instances engineInstances;

static int tektagonPlatformInit()
{
	int status = 0;
	status = initializeEngines(/*&engineInstances*/);
	if(status)
		return status;

	status = initializeManifestProcessor();
	if(status)
			return status;
	printk("Platform Init\r\n");
	return status;
}

void main(void)
{
	int status = 0;
	printk("\r\n *** Tektagon OE version 1.0 ***\r\n");
	// clear internal logging
	//status = debug_log_clear();
	//if(status)
		//return status;

	status = tektagonPlatformInit();

	if(status)
			return status;

	// Process PFM from Flash into Cerberus library
	status = processPfmFlashManifest();
	
	if(status){
		printk("System Lock \n");
		return status;
	}else{
		printk("Verification Complete Release BMC\n");
		pfr_bmc_rst_flash(1);
		pfr_bmc_rst_enable_ctrl(false);
		BmcBootRelease();
	}
}
