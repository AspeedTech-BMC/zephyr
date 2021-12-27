/*
 * definitions.h
 *
 *  Created on: Dec 15, 2021
 *      Author: presannar
 */

#ifndef ZEPHYR_TEKTAGON_SRC_INCLUDE_DEFINITIONS_H_
#define ZEPHYR_TEKTAGON_SRC_INCLUDE_DEFINITIONS_H_

#include <zephyr.h>

#define SPI_FLASH_SIZE 0x1000000
#define PFM_FLASH_MANIFEST_ADDRESS 0x7c00000

#define	HASH_ENGINE_NAME	wrapper

#define	HASH_ENGINE_INIT_FUNC_DEF(name)	hash_ ## name ## _init
#define	HASH_ENGINE_INIT_FUNC(name)		HASH_ENGINE_INIT_FUNC_DEF(name)
#define	HASH_ENGINE_INIT				HASH_ENGINE_INIT_FUNC(HASH_ENGINE_NAME)

#ifdef CONFIG_DEBUG
#define DEBUG_PRINTF(val) printk(val)
#else
#define DEBUG_PRINTF(val)
#endif

enum {
	DEBUG_LOG_TEKTAGON_MAIN = 0x20,
};

/**
 * Logging messages for command handling.
 *
 */
/*enum {

};*/

struct pfmInfo {
	// TBD Fix Void* to point to the correct structure
	void *hash;											/**< Hashing engine for validation. */
	struct pfm_flash *pfmFlash;							/**< PFM instance under test. */
	struct signature_verification *sigVerification;		/**< PFM signature verification. */
	struct flash_master *flashMaster;					/**< Flash master for the PFM flash. */
	struct spi_flash *spiFlash;							/**< Flash where the PFM is stored. */
	uint32_t address;									/**< Base address of the PFM. */
	uint8_t signature[256];								/**< Buffer for the manifest signature. */
	uint8_t platform_id[256];							/**< Cache for the platform ID. */
};

#endif /* ZEPHYR_TEKTAGON_SRC_INCLUDE_DEFINITIONS_H_ */
