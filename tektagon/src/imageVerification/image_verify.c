/*
 * image_verify.c
 *
 *  Created on: Dec 13, 2021
 *      Author: presannar
 */

#include <common/signature_verification.h>

#include <flash/flash_master.h>
#include "image_verify.h"
#include "common/common.h"

int rsa_public_key_init(struct rsa_public_key *public_key){
	struct flash *flash_device = getFlashDeviceInstance();
	struct manifest_flash manifestFlash;
	uint32_t public_key_offset,exponent_offset;
	uint16_t module_length;
	uint8_t exponent_length;

	flash_device->read(flash_device, PFM_FLASH_MANIFEST_ADDRESS,&manifestFlash.header,sizeof (manifestFlash.header));
	flash_device->read(flash_device, PFM_FLASH_MANIFEST_ADDRESS+manifestFlash.header.length,&module_length,sizeof (module_length));
	
	public_key_offset = PFM_FLASH_MANIFEST_ADDRESS + manifestFlash.header.length + sizeof (module_length);
	public_key->mod_length = module_length;

	uint8_t buf[public_key->mod_length];

	flash_device->read(flash_device, public_key_offset ,public_key->modulus, public_key->mod_length);
	//memcpy(public_key->modulus,buf,public_key->mod_length);

	exponent_offset = public_key_offset + public_key->mod_length;
	flash_device->read(flash_device, exponent_offset ,&exponent_length,sizeof(exponent_length));
	flash_device->read(flash_device,exponent_offset+sizeof(exponent_length) ,&public_key->exponent,exponent_length);
	
	return 0;
}

int rsa_verify_signature (struct signature_verification *verification,
	const uint8_t *digest, size_t length, const uint8_t *signature, size_t sig_length)
{
	struct rsa_public_key rsa_public;
	rsa_public_key_init(&rsa_public);
	struct rsa_engine_wrapper *rsa = getRsaEngineInstance();

	return rsa->base.sig_verify (&rsa->base, &rsa_public, signature, sig_length, digest, length);
}

int signature_verification_init(struct signature_verification *verification)
{
	int status = 0;
	memset (verification, 0, sizeof (struct signature_verification));

	verification->verify_signature = rsa_verify_signature;

	return status;
}


