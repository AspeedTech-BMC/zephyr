/*
 * manifestProcessor.c
 *
 *  Created on: Dec 15, 2021
 *      Author: presannar
 */
#include <assert.h>

#include "include/definitions.h"
#include "manifestProcessor.h"
#include "common/common.h"


int initializeManifestProcessor()
{
	int status = 0;

	status = manifest_flash_init(getManifestFlashInstance(), getFlashDeviceInstance(), PFM_FLASH_MANIFEST_ADDRESS, PFM_V2_MAGIC_NUM);
	if(status)
		return status;

	status = pfm_manager_flash_init(getPfmManagerFlashInstance(), getPfmFlashInstance(), getPfmFlashInstance(),
			getHostStateManagerInstance(), getHashEngineInstance(), getSignatureVerificationInstance());

	return status;
}

int processPfmFlashManifest()
{
	int status = 0;
	uint8_t *hashStorage = getNewHashStorage();

	status = manifest_flash_verify(getManifestFlashInstance(), getHashEngineInstance(), getSignatureVerificationInstance(), hashStorage, sizeof(hashStorage));
	if(status)
		return status;

	/*if(getManifestFlashInstance()->manifest_valid)
	{
	}*/

	return status;
}
