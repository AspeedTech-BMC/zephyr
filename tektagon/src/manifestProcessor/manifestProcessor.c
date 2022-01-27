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

int recovery_header_magic_num_check(uint8_t *magic_num){
	if (memcmp(magic_num, recovery_image_magic_num, sizeof(recovery_image_magic_num))){
		return 0;
	}else{
		printk("Recovery Header Magic Number not Match.\r\n");
		return 1;
	}
		
}

int recovery_verification(struct flash *flash,struct hash_engine *hash,struct rsa_engine *rsa,
							struct rsa_public_key *pub_key,uint8_t *hash_out,size_t hash_length){
	int status = 0;
	struct recovery_image_header recovery_header;
	uint32_t recovery_signature_offset;
	int signature_length;

	flash->read (flash, RECOVERY_IMAGE_BASE_ADDRESS, &recovery_header,sizeof (recovery_header));

	status = recovery_header_magic_num_check(recovery_header.magic_num);
	// if (status)
	// 	return status;

	signature_length =  *((uint32_t*)  recovery_header.sig_length);
	uint8_t signature[signature_length];
	
	recovery_signature_offset = RECOVERY_IMAGE_BASE_ADDRESS + *((uint32_t*)  recovery_header.image_length) - *((uint32_t*)  recovery_header.sig_length);

	flash->read (flash, recovery_signature_offset, signature,signature_length);

	status = flash_verify_contents(flash,
									RECOVERY_IMAGE_BASE_ADDRESS,
									*((uint32_t*)  recovery_header.image_length) - *((uint32_t*)  recovery_header.sig_length),
									getHashEngineInstance(),
									1,
									getRsaEngineInstance(),
									signature,
									256,
									pub_key,
									hash_out,
									256
									);
	return status;
}

int recovery_action(struct flash *flash,uint8_t *recovery_address){
	int status = 0;
	struct recovery_image_header recovery_header;
	struct recovery_image recovery_info;
	uint32_t image_offset = 0, recovery_signature_offset, erase_block_offset;
	int match_flag = 0;
	char buf[2048];
	uint32_t recovery_read_offset, active_write_offset;
	int block_count = 0;
	int page_count = 0;
	flash->read (flash, RECOVERY_IMAGE_BASE_ADDRESS, &recovery_header,sizeof (recovery_header));

	image_offset = RECOVERY_IMAGE_BASE_ADDRESS + recovery_header.header_length;
	recovery_signature_offset = RECOVERY_IMAGE_BASE_ADDRESS + *((uint32_t*)  recovery_header.image_length) - *((uint32_t*)  recovery_header.sig_length);

	do{
		flash->read (flash, image_offset, &recovery_info,sizeof (recovery_info));

		image_offset += sizeof(recovery_info); // image data start

		if(!memcmp(recovery_address, recovery_info.address, sizeof(recovery_info.address))){
			printk("Start Recovery\n");

			match_flag = 1;
			//do recovery here;
			recovery_read_offset = image_offset;
			active_write_offset = *((uint32_t*)recovery_address);

			printk("recovery offset:%x\n", active_write_offset);

			page_count = *((uint32_t*)recovery_info.image_length) / sizeof(buf);
			block_count = *((uint32_t*)recovery_info.image_length) / 1024 / 64;

			erase_block_offset = active_write_offset;
			for(int i = 0; i < block_count; i++){
				flash->block_erase(flash, erase_block_offset);
				erase_block_offset += 1024 * 64; // 64K
			}

			for(int page = 0; page < page_count; page++){
				flash->read (flash, recovery_read_offset + page * sizeof(buf), buf,sizeof(buf));
				flash->write(flash, active_write_offset + page * sizeof(buf), buf, sizeof(buf));
			}		
			printk("recovery offset:%x recovery successful\n", active_write_offset);
			break;
		}
		image_offset += *((uint32_t*)recovery_info.image_length);// image data end
	}while (image_offset != recovery_signature_offset);

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
	printk("Manifest Verification\n");
	status = manifest_flash_verify(manifest_flash, getHashEngineInstance(),
			getSignatureVerificationInstance(), hashStorage, hashStorageLength);

	if(true == manifest_flash->manifest_valid)
	{
		printk("Manifest Verificaation Successful\n");
		uint8_t flash_signature[256];
		uint32_t firmware_offset; 
		int i;
		struct manifest_toc_entry entry;
		struct manifest_platform_id plat_id_header;
		struct manifest_fw_element_header fw_header;
		struct manifest_fw_elements fw_element;
		manifest_flash->toc_hash_length = SHA256_HASH_LENGTH;
		int recovery_verify_result = 0;
		int recovery_verify_flag = 0;

		//12 bytes
		firmware_offset = manifest_flash->addr + sizeof (manifest_flash->header);

		// 4 bytes
		manifest_flash->flash->read (manifest_flash->flash, firmware_offset, (uint8_t*) &manifest_flash->toc_header,
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


		firmware_offset += sizeof (allow_firmware);

		for(int rw_index = 0; rw_index < allow_firmware.header.rw_count; rw_index++){
			firmware_offset += sizeof(struct allowable_rw_region);
		}

		for(int verify_index=0; verify_index < allow_firmware.header.image_count; verify_index++){
			struct signature_firmware_region firmware_info;

			memset(&firmware_info, 0, sizeof(firmware_info));
			manifest_flash->flash->read (manifest_flash->flash, firmware_offset, &firmware_info,sizeof (struct signature_firmware_region));
		
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

			if(status){
				printk("Active Verification Fail\n");
				printk("start_address:%x\n",*((uint32_t*) firmware_info.start_address));
				printk("end_address:%x\n",*((uint32_t*) firmware_info.end_address));
			
				if(recovery_verify_flag == 0){
					printk("Recovery Verification\n");
					recovery_verify_result = recovery_verification(manifest_flash->flash,
																	getHashEngineInstance(),
																	getRsaEngineInstance(),
																	&pub_key,
																	hashStorage,
																	256);
					recovery_verify_flag++;
					
					if(recovery_verify_result){
						printk("Recovery Verification Fail\n");
						return recovery_verify_result;
					}else{
						printk("Recovery Verification Successful\n");
					}
				}
				status = recovery_action(manifest_flash->flash,firmware_info.start_address);
			}else{
				printk("Active Region offset %x verify successful\n", *((uint32_t*) firmware_info.start_address));
			}
			firmware_offset += sizeof (firmware_info);
		}
	}

	return status;
}
