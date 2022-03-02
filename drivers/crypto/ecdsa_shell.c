/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <shell/shell.h>
#include "mbedtls/ecdsa.h"

struct ecdsa_testvec {
	const unsigned char *Qx;
	const unsigned char *Qy;
	const unsigned char *d;
	const unsigned char *r;
	const unsigned char *s;
	const unsigned char *m;
	unsigned int m_size;
};

static const struct ecdsa_testvec secp384r1[] = {
	{
		/* secp384r1(sha256) */
		/* public key */
		.Qx =
		"\xC5\xCA\xAD\xEC\x40\x8D\x51\x2D\xF9\xC2\x30\xEC\x6A\x1E\x88\x65"
		"\x91\x7B\x87\xBE\x26\x7C\x15\x00\x9F\x06\x19\x9A\x19\x31\x17\xCF"
		"\xA7\x21\xBE\xF8\x7A\x63\xC5\xF4\xE4\x61\x59\xBD\x46\x3F\x48\xAE",
		.Qy =
		"\x5D\x66\xA0\x1C\x9F\xDF\x69\xB0\x4C\x09\xFF\xE5\x54\x7F\xA8\x05"
		"\x9B\xA4\x94\x07\xB8\x69\x05\xC0\xF7\xCB\x08\xAA\x31\x16\x74\xE9"
		"\x1A\x9B\x33\x53\x0F\x75\x8A\x9E\x69\x0D\x5F\xC3\x63\xB2\xB0\x27",
		/* signature */
		.r =
		"\x53\x3E\xE5\xBF\x40\xEC\x2D\x67\x98\x8B\x77\xF3\x17\x48\x9B\xB6"
		"\xDF\x95\x29\x25\xC7\x09\xFC\x03\x81\x11\x1A\x59\x56\xF2\xD7\x58"
		"\x11\x0E\x59\xD3\xD7\xC1\x72\x9E\x2C\x0D\x70\xEA\xF7\x73\xE6\x12",
		.s =
		"\xA0\xFB\xEC\xDD\x33\x5B\x83\xC1\x68\xC6\x50\x52\xFA\x34\x0C\x31"
		"\xDE\x0C\x9F\xFD\xC5\x59\x4A\x64\x13\xB7\x35\x94\xD3\x55\x47\x31"
		"\x4D\xF8\x45\x9C\xDC\x7A\x73\x75\xDD\x48\xD7\x45\x9E\x57\xC3\x8D",
		/* sha1 digest */
		.m =
		"\xDD\xA6\x27\x1B\xA0\xDC\xD9\xEE\x62\x4F\x18\xDF\x8F\x62\x9B\x6B"
		"\xAE\x08\xD2\x08\xB0\x17\xB5\x0E\x06\xE6\x12\x8D\x47\xE2\x86\x41",
		.m_size = 32
	}
};

static void dump_buf(const struct shell *shell, const char *title,
					 const unsigned char *buf, size_t len)
{
	size_t i;

	shell_fprintf(shell, SHELL_NORMAL, "%s", title);
	for (i = 0; i < len; i++)
		shell_fprintf(shell, SHELL_NORMAL, "%c%c", "0123456789ABCDEF"[buf[i] / 16],
					  "0123456789ABCDEF"[buf[i] % 16]);
	shell_fprintf(shell, SHELL_NORMAL, "\n");
}

static void dump_pubkey(const struct shell *shell, const char *title, mbedtls_ecdsa_context *key)
{
	unsigned char buf[300];
	size_t len;

	if (mbedtls_ecp_point_write_binary(&key->grp, &key->Q,
									   MBEDTLS_ECP_PF_UNCOMPRESSED, &len, buf, sizeof(buf)) != 0) {
		shell_fprintf(shell, SHELL_NORMAL, "internal error\n");
		return;
	}
	dump_buf(shell, title, buf, len);

	mbedtls_mpi_write_binary(&key->Q.X, buf, 48);
	dump_buf(shell, "  + Qx: ", buf, 48);
	mbedtls_mpi_write_binary(&key->Q.Y, buf, 48);
	dump_buf(shell, "  + Qy: ", buf, 48);
	mbedtls_mpi_write_binary(&key->Q.Z, buf, 48);
	dump_buf(shell, "  + Qz: ", buf, 48);

}

static int mbedtls_ecdsa_test(const struct shell *shell, int argc, char *argv[])
{
	mbedtls_ecdsa_context ctx_verify;
	int ret = 0;
	int i;
	mbedtls_mpi r, s;
	unsigned char hash[32];
	char z = 1;

	for (i = 0; i < ARRAY_SIZE(secp384r1); i++) {
		mbedtls_ecdsa_init(&ctx_verify);
		mbedtls_mpi_init(&r);
		mbedtls_mpi_init(&s);
		mbedtls_mpi_read_binary(&ctx_verify.Q.X, secp384r1[i].Qx, 48);
		mbedtls_mpi_read_binary(&ctx_verify.Q.Y, secp384r1[i].Qy, 48);
		mbedtls_mpi_read_binary(&ctx_verify.Q.Z, &z, 1);
		mbedtls_mpi_read_binary(&r, secp384r1[i].r, 48);
		mbedtls_mpi_read_binary(&s, secp384r1[i].s, 48);

		shell_fprintf(shell, SHELL_NORMAL, " start load curve\n");
		mbedtls_ecp_group_load(&ctx_verify.grp, MBEDTLS_ECP_DP_SECP384R1);
		shell_fprintf(shell, SHELL_NORMAL, " start verify\n");
		dump_pubkey(shell, "  + Public key: ", &ctx_verify);
		shell_fprintf(shell, SHELL_NORMAL, " signature\n");
		dump_buf(shell, "  + r: ", secp384r1[i].r, 48);
		dump_buf(shell, "  + s: ", secp384r1[i].s, 48);
		memcpy(hash, secp384r1[i].m, secp384r1[i].m_size);

		ret = mbedtls_ecdsa_verify(&ctx_verify.grp, hash, secp384r1[i].m_size,
								   &ctx_verify.Q, &r, &s);
		if (ret != 0) {
			shell_fprintf(shell, SHELL_NORMAL, " failed! mbedtls_ecdsa_verify returned %d\n", ret);
		}
	}

	shell_fprintf(shell, SHELL_NORMAL, " ok\n");
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ecdsa_cmds,
							   SHELL_CMD_ARG(test, NULL, "", mbedtls_ecdsa_test, 1, 0),
							   SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ecdsa, &ecdsa_cmds, "ECDSA shell commands", NULL);
