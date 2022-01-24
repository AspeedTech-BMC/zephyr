/*
 * manifestProcessor.h
 *
 *  Created on: Dec 15, 2021
 *      Author: presannar
 */

#ifndef ZEPHYR_TEKTAGON_SRC_MANIFESTPROCESSOR_MANIFESTPROCESSOR_H_
#define ZEPHYR_TEKTAGON_SRC_MANIFESTPROCESSOR_MANIFESTPROCESSOR_H_

#include <zephyr.h>

int initializeManifestProcessor();
int processPfmFlashManifest();

struct manifest_flash_device{
    uint8_t blank;
    uint8_t fw_count;
    uint16_t reserved;
};

struct manifest_fw_element_header{
    uint8_t version_count;
    uint8_t fw_id_length;
    uint8_t fw_flags;
    uint8_t reserved;
};

struct manifest_fw_elements{
    struct manifest_fw_element_header header;
    char *platformid;
    uint8_t alignment;
};

struct allowable_fw_header{
    uint8_t image_count;
    uint8_t rw_count;
    uint8_t version_length;
    uint8_t reserved;
};

struct allowable_fw{
    struct allowable_fw_header header;
    uint32_t version_address;
    uint8_t version_id[6];
    uint16_t alignment;
};

struct allowable_rw_region{
    uint8_t flag;
    uint8_t reserved[3];
    uint32_t start_address;
    uint32_t end_address;
};

struct signature_firmware_region{
    uint8_t hash_type;
    uint8_t region_count;
    uint8_t flag;
    uint8_t reserved;
    uint8_t signature[256];
    char public_key[450];  //need to discuss
    uint8_t start_address[4];
    uint8_t end_address[4];
};

#endif /* ZEPHYR_TEKTAGON_SRC_MANIFESTPROCESSOR_MANIFESTPROCESSOR_H_ */
