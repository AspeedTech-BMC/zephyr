/*
 * engine_manager.c
 *
 *  Created on: Dec 13, 2021
 *      Author: presannar
 */

#include <assert.h>

#include "engine_manager.h"
#include "include/definitions.h"
#include "common/common.h"


uint8_t signature[RSA_MAX_KEY_LENGTH];		/**< Buffer for the manifest signature. */
uint8_t platform_id[256];					/**< Cache for the platform ID. */

static int crypto_engine_init(/*struct engine_instances *engineInstances*/)
{
	int status = 0;

	status = hash_wrapper_init (getHashEngineInstance());
	assert(status == 0);
	status = rsa_aspeed_init(getRsaEngineInstance());
	assert(status == 0);

/*	engineInstances->hashEngine = getHashEngineInstance();
	engineInstances->rsaEngine = getRsaEngineInstance();

	engineInstances->hashEngine = getHashEngineInstance();
	engineInstances->flashMaster = getFlashMasterInstance();
	engineInstances->spiFlash = getSpiFlashInstance();
	engineInstances->pfmFlash = getPfmFlashInstance();*/

	return status;
}

static int flash_initialize(/*struct engine_instances *engineInstances*/)
{
	int status = 0;

	status = flash_wrapper_init(getSpiEngineWrapper(), getFlashEngineWrapper());
	if(status)
		return status;

/*	engineInstances->spiEngineWrapper = getSpiEngineWrapper();
	engineInstances->flashEngineWrapper = getFlashEngineWrapper();*/

	return status;
}

int initializeEngines(/*struct engine_instances *engineInstances*/)
{
	int status = 0;

	printk("Inside Initialize Engines\r\n");

	status = crypto_engine_init (/*engineInstances*/);
	assert(status == 0);
	status = signature_verification_init (getSignatureVerificationInstance());
	assert(status == 0);
	status = flash_master_wrapper_init (getFlashMasterInstance());
	assert(status == 0);
	status = spi_flash_init (getSpiFlashInstance(), getFlashMasterInstance());
	assert(status == 0);
	status = spi_flash_initialize_device(getSpiFlashInstance(), getFlashMasterInstance(), true, false, false, false);
	assert(status == 0);
	status = pfm_flash_init(getPfmFlashInstance(), getFlashDeviceInstance(), getHashEngineInstance(), PFM_FLASH_MANIFEST_ADDRESS, signature, RSA_MAX_KEY_LENGTH, platform_id, sizeof(platform_id));
	assert(status == 0);

	status = flash_initialize(/*engineInstances*/);
	if(status)
		return status;

	printk("After InitializeEngines\r\n");

	return status;
}
