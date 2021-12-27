/*
 * common.h
 *
 *  Created on: Dec 22, 2021
 *      Author: presannar
 */

#ifndef ZEPHYR_TEKTAGON_SRC_COMMON_COMMON_H_
#define ZEPHYR_TEKTAGON_SRC_COMMON_COMMON_H_

/* Cerberus Includes*/
#include <common/signature_verification.h>

#include <crypto/aes.h>
#include <crypto/base64.h>
#include <crypto/ecc.h>
#include <crypto/hash.h>
#include <crypto/rng.h>
#include <crypto/rsa.h>
#include <crypto/x509.h>

#include <crypto/rsa_wrapper.h>

#include <flash/flash.h>
#include <flash/flash_master.h>
#include <flash/spi_flash.h>

#include <flash/flash_wrapper.h>


#include <keystore/keystore.h>

#include <manifest/manifest.h>
#include <manifest/pfm/pfm_flash.h>
#include <manifest/pfm/pfm_manager_flash.h>

struct flash *getFlashDeviceInstance();
struct flash_master *getFlashMasterInstance();
struct hash_engine *getHashEngineInstance();
struct host_state_manager *getHostStateManagerInstance();
struct pfm_flash *getPfmFlashInstance();
struct signature_verification *getSignatureVerificationInstance();
struct spi_flash *getSpiFlashInstance();
struct rsa_engine_wrapper *getRsaEngineInstance();

#endif /* ZEPHYR_TEKTAGON_SRC_COMMON_COMMON_H_ */
