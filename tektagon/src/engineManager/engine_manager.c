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
#include "imageVerification/image_verify.h"

uint8_t signature[RSA_MAX_KEY_LENGTH];		/**< Buffer for the manifest signature. */
uint8_t platform_id[256];					/**< Cache for the platform ID. */

static int crypto_engine_init(/*struct engine_instances *engineInstances*/)
{
	int status = 0;

	status = hash_wrapper_init (getHashEngineInstance());
	assert(status == 0);
	status = rsa_wrapper_init(getRsaEngineInstance());
	assert(status == 0);

	return status;
}

static int flash_initialize(/*struct engine_instances *engineInstances*/)
{
	int status = 0;

	status = flash_wrapper_init(getSpiEngineWrapper(), getFlashEngineWrapper());
	if(status)
		return status;

	return status;
}

int initializeEngines(/*struct engine_instances *engineInstances*/)
{
	int status = 0;
	
	status = flash_master_wrapper_init (getFlashMasterInstance());
	assert(status == 0);
	status = flash_initialize(/*engineInstances*/);
	assert(status == 0);
	status = crypto_engine_init (/*engineInstances*/);
	assert(status == 0);
	status = signature_verification_init (getSignatureVerificationInstance());
	assert(status == 0);

	status = pfm_flash_init(getPfmFlashInstance(), getFlashDeviceInstance(), getHashEngineInstance(), PFM_FLASH_MANIFEST_ADDRESS, signature, RSA_MAX_KEY_LENGTH, platform_id, sizeof(platform_id));
	assert(status == 0);
	
	if(status)
		return status;

	return status;
}
