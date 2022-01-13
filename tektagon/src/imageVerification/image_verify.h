/*
 * image_verify.h
 *
 *  Created on: Dec 13, 2021
 *      Author: presannar
 */

#ifndef ZEPHYR_TEKTAGON_SRC_IMAGEVERIFICATION_IMAGE_VERIFY_H_
#define ZEPHYR_TEKTAGON_SRC_IMAGEVERIFICATION_IMAGE_VERIFY_H_

#include <zephyr.h>
#include <crypto/signature_verification_rsa_wrapper.h>
#include <crypto/rsa.h>
#include <include/definitions.h>
void handleVerifyEntryState(/* TBD */);
void handleVerifyExitState(/* TBD */);

int performimageVerification();
int signature_verification_init(struct signature_verification *verification);
int rsa_public_key_init(struct rsa_public_key *public_key);
#endif /* ZEPHYR_TEKTAGON_SRC_IMAGEVERIFICATION_IMAGE_VERIFY_H_ */
