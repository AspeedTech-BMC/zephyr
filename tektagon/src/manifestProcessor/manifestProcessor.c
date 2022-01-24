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
#include "firmware/app_image.h"

int initializeManifestProcessor()
{
	int status = 0;

	status = manifest_flash_init(getManifestFlashInstance(), getFlashDeviceInstance(), PFM_FLASH_MANIFEST_ADDRESS, PFM_V2_MAGIC_NUM);
	if(status)
		return status;
	
	//status = pfm_manager_flash_init(getPfmManagerFlashInstance(), getPfmFlashInstance(), getPfmFlashInstance(),
			//getHostStateManagerInstance(), getHashEngineInstance(), getSignatureVerificationInstance());
	
	return status;
}

int processPfmFlashManifest()
{
	int status = 0;
	uint8_t *hashStorage = getNewHashStorage();
	struct manifest_flash *manifest_flash = getManifestFlashInstance();
	struct rsa_public_key pub_key;

	status = rsa_public_key_init(&pub_key);
	if(status)
		return status;

	printk("After successful Pubkey Read\r\n");

	status = manifest_flash_verify(manifest_flash, getHashEngineInstance(),
			getSignatureVerificationInstance(), hashStorage, hashStorageLength);

	if(true == manifest_flash->manifest_valid)
	{
		uint8_t flash_signature[256];

		uint32_t firmware_offset; 
		int i;
		struct manifest_toc_entry entry;
		struct manifest_platform_id plat_id_header;
		struct manifest_fw_element_header fw_header;
		struct manifest_fw_elements fw_element;
		manifest_flash->toc_hash_length = SHA256_HASH_LENGTH;

		//12 bytes
		firmware_offset = manifest_flash->addr + sizeof (manifest_flash->header);

		// 4 bytes
		status = manifest_flash->flash->read (manifest_flash->flash, firmware_offset, (uint8_t*) &manifest_flash->toc_header,
			sizeof (manifest_flash->toc_header));
		firmware_offset += sizeof (manifest_flash->toc_header);

		// 8 * 4 bytes
		i = 0;
		do {
			firmware_offset += sizeof (entry);
			i++;
		} while ((i < manifest_flash->toc_header.entry_count));

		// 32 * 4 bytes
		firmware_offset += (manifest_flash->toc_header.hash_count * manifest_flash->toc_hash_length);

		//32 bytes
		firmware_offset += manifest_flash->toc_hash_length;

		manifest_flash->flash->read (manifest_flash->flash, firmware_offset, &plat_id_header,
			sizeof (plat_id_header));

		//4 bytes
		firmware_offset += sizeof (plat_id_header);

		//10 bytes
		firmware_offset += plat_id_header.id_length;
		
		//alignment 2 bytes
		firmware_offset += 2;

		//flash device 4 bytes
		firmware_offset += sizeof(struct manifest_flash_device);

		manifest_flash->flash->read (manifest_flash->flash, firmware_offset, &fw_header,
			sizeof (fw_header));			

		//fw_elements 8 bytes
		firmware_offset += sizeof (fw_header) + fw_header.fw_id_length + sizeof(fw_element.alignment);
		

		struct allowable_fw allow_firmware;

		manifest_flash->flash->read (manifest_flash->flash, firmware_offset, &allow_firmware,
			sizeof (allow_firmware));

		printk("image_count:%d\n", allow_firmware.header.image_count);
		firmware_offset += sizeof (allow_firmware);

		for(int rw_index = 0; rw_index < allow_firmware.header.rw_count; rw_index++){
			firmware_offset += sizeof(struct allowable_rw_region);
		}
		



		for(int verify_index=0; verify_index < allow_firmware.header.image_count; verify_index++){
			struct signature_firmware_region firmware_info;

			memset(&firmware_info, 0, sizeof(firmware_info));
			manifest_flash->flash->read (manifest_flash->flash, firmware_offset, &firmware_info,sizeof (struct signature_firmware_region));
			printk("sizeof (firmware_info):%x\n",sizeof (struct signature_firmware_region));
			printk("firmware_offset:%x\n",firmware_offset);
			

			// uint32_t start_address, end_address;
			// uint32_t start_address = firmware_info.start_address[0] | (firmware_info.start_address[1] << 8) | (firmware_info.start_address[2] << 16) | (firmware_info.start_address[3] << 24);
			// uint32_t end_address = firmware_info.end_address[0] | (firmware_info.end_address[1] << 8) | (firmware_info.end_address[2] << 16) | (firmware_info.end_address[3] << 24);
			// uint32_t start_address = *((uint32_t*) firmware_info.start_address);

			printk("firmware_info.start_address:%x\n",*((uint32_t*) firmware_info.start_address));
			printk("firmware_info.end_address:%x\n",*((uint32_t*) firmware_info.end_address));
			printk("length:%x\n", *((uint32_t*) firmware_info.end_address)-*((uint32_t*) firmware_info.start_address));
			status = flash_verify_contents(manifest_flash->flash,
											*((uint32_t*) firmware_info.start_address),
											*((uint32_t*) firmware_info.end_address)-*((uint32_t*) firmware_info.start_address)+sizeof(uint8_t),
											getHashEngineInstance(),
											1,
											getRsaEngineInstance(),
											firmware_info.signature,
											256,
											&pub_key,
											hashStorage,
											256
											);
			printk("verify status:%x\n", status);

			firmware_offset += sizeof (firmware_info);
		}


		// status = app_image_verification(getFlashDeviceInstance(), 0, getHashEngineInstance(),
		// 		&(getRsaEngineInstance()->base), &pub_key, NULL, RSA_MAX_KEY_LENGTH);
		
		// printk("app_image_verification status:%x\n", status);
		// if(status)
		// 	return status;
	}

	return status;
}
