/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmd_util.h"
#include "cmd_ce.h"
#include "driver/chip/hal_crypto.h"
#if (__CONFIG_CHIP_ARCH_VER == 2)
#include "driver/chip/hal_trng.h"
#endif
#define CE_PLAIN_MAX_LEN 1024
#define CE_CIPHER_MAX_LEN (CE_PLAIN_MAX_LEN << 2)

static void cipher_str_to_arr(char *cipher_str, uint8_t* cipher_buf)
{
	for(int i=0; i<cmd_strlen(cipher_str); i+=2) {
		if(cipher_str[i] <= '9') {
			cipher_buf[i>>1] = (cipher_str[i]-'0') << 4;
		} else if(cipher_str[i] <= 'F') {
			cipher_buf[i>>1] = (cipher_str[i]-55) << 4;
		} else {
			cipher_buf[i>>1] = (cipher_str[i]-87) << 4;
		}

		if(cipher_str[i+1] <= '9') {
			cipher_buf[i>>1] += cipher_str[i+1]-'0';
		} else if(cipher_str[i+1] <= 'F') {
			cipher_buf[i>>1] += cipher_str[i+1]-55;
		} else {
			cipher_buf[i>>1] += cipher_str[i+1]-87;
		}
	}
}

static enum cmd_status cmd_encrypto_aes_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[33];
	char* plain_str;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t plain_len = 0;
	uint32_t cipher_len = 0;
	CE_AES_Config aes_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }
	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s p=%s", mode_str, key_str, plain_str);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
        ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	memset(&aes_cfg, 0, sizeof(aes_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		aes_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		aes_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}

	key_len = cmd_strlen(key_str);
	if(16 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_128BITS;
	} else if(24 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_192BITS;
	} else if(32 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_256BITS;
	} else {
		CMD_ERR("invalid param:  key len %d != (16/24/32)\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	aes_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	sprintf((char*)aes_cfg.key, key_str);

	plain_len = cmd_strlen(plain_str);
	cipher_len = (plain_len + 15) & (~0xF);
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_AES_Encrypt(&aes_cfg, (uint8_t*)plain_str, (uint8_t*)cipher_buf, plain_len) != HAL_OK) {
		CMD_ERR("AES encrypt failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_cipher;
	}
	printf("ciphertest(hex):");
	cmd_print_uint8_array((uint8_t *)cipher_buf, cipher_len);

out_cipher:
	cmd_free(cipher_buf);
out_plain:
    cmd_free(plain_str);
	return ret;
}

static enum cmd_status cmd_decrypto_aes_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[33];
	char* cipher_str;
	uint8_t *plain_buf;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t cipher_len = 0;
	CE_AES_Config aes_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    cipher_str = (char*)cmd_malloc(CE_CIPHER_MAX_LEN);
    if(cipher_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_cipher;
    }

	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s c=%s", mode_str, key_str, cipher_str);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	memset(&aes_cfg, 0, sizeof(aes_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		aes_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		aes_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}

	key_len = cmd_strlen(key_str);
	if(16 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_128BITS;
	} else if(24 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_192BITS;
	} else if(32 == key_len) {
		aes_cfg.keysize = CE_CTL_AES_KEYSIZE_256BITS;
	} else {
		CMD_ERR("invalid param:  key len %d != (16/24/32)\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	aes_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	sprintf((char*)aes_cfg.key, key_str);

	cipher_len = cmd_strlen(cipher_str);
	if((cipher_len & 0xF) != 0) {
		CMD_ERR("invalid param:  cipher len %d != (16*n)\n", cipher_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	cipher_len = cipher_len >> 1;
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	cipher_str_to_arr(cipher_str, cipher_buf);
	plain_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_AES_Decrypt(&aes_cfg, (uint8_t*)cipher_buf, (uint8_t*)plain_buf, cipher_len) != HAL_OK) {
		CMD_ERR("AES encrypt failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("plaintest:");
	cmd_raw_mode_enable();
	cmd_raw_mode_write(plain_buf, cipher_len);
	cmd_raw_mode_disable();
	printf("\n\n");

out_plain:
	cmd_free(cipher_buf);
	cmd_free(plain_buf);
out_cipher:
    cmd_free(cipher_str);
	return ret;
}

static enum cmd_status cmd_encrypto_des_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[33];
	char* plain_str = NULL;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t plain_len = 0;
	uint32_t cipher_len = 0;
	CE_DES_Config des_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }
    memset(plain_str, 0, CE_PLAIN_MAX_LEN);

	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s p=%s", mode_str, key_str, plain_str);
	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	memset(&des_cfg, 0, sizeof(des_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}

	key_len = cmd_strlen(key_str);
	if(8 != key_len) {
		CMD_ERR("invalid param:  key len %d != 8\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	des_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	sprintf((char*)des_cfg.key, key_str);

	plain_len = cmd_strlen(plain_str);
    plain_len = plain_len > 8 ? plain_len : 8;
	cipher_len = (plain_len + 7) & (~0x7);
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_DES_Encrypt(&des_cfg, (uint8_t*)plain_str, (uint8_t*)cipher_buf, plain_len) != HAL_OK) {
		CMD_ERR("DES encrypt failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_cipher;
	}
	printf("ciphertest(hex):");
	cmd_print_uint8_array((uint8_t *)cipher_buf, cipher_len);

out_cipher:
	cmd_free(cipher_buf);
out_plain:
    cmd_free(plain_str);
	return ret;
}

static enum cmd_status cmd_decrypto_des_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[33];
	char* cipher_str;
	uint8_t *plain_buf;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t cipher_len = 0;
	CE_DES_Config des_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    cipher_str = (char*)cmd_malloc(CE_CIPHER_MAX_LEN);
    if(cipher_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_cipher;
    }
    memset(cipher_str, 0, CE_CIPHER_MAX_LEN);

	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s c=%s", mode_str, key_str, cipher_str);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	memset(&des_cfg, 0, sizeof(des_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}

	key_len = cmd_strlen(key_str);
	if(8 != key_len) {
		CMD_ERR("invalid param:  key len %d != 8\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	des_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	sprintf((char*)des_cfg.key, key_str);

	cipher_len = cmd_strlen(cipher_str);
	cipher_len = cipher_len >> 1;
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	cipher_str_to_arr(cipher_str, cipher_buf);
	plain_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_DES_Decrypt(&des_cfg, (uint8_t*)cipher_buf, (uint8_t*)plain_buf, cipher_len) != HAL_OK) {
		CMD_ERR("DES encrypt failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("plaintest:");
	cmd_raw_mode_enable();
	cmd_raw_mode_write(plain_buf, cipher_len);
	cmd_raw_mode_disable();
	printf("\n\n");

out_plain:
    cmd_free(cipher_buf);
    cmd_free(plain_buf);
out_cipher:
    cmd_free(cipher_str);
    return ret;
}

static enum cmd_status cmd_encrypto_3des_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[33];
	char* plain_str;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t plain_len = 0;
	uint32_t cipher_len = 0;
	CE_3DES_Config des_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }
    memset(plain_str, 0, CE_PLAIN_MAX_LEN);

	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s p=%s", mode_str, key_str, plain_str);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	memset(&des_cfg, 0, sizeof(des_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}

	key_len = cmd_strlen(key_str);
	if(24 != key_len) {
		CMD_ERR("invalid param:  key len %d != 24\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	des_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	sprintf((char*)des_cfg.key, key_str);

	plain_len = cmd_strlen(plain_str);
    plain_len = plain_len > 8 ? plain_len : 8;
	cipher_len = (plain_len + 7) & (~0x7);
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_3DES_Encrypt(&des_cfg, (uint8_t*)plain_str, (uint8_t*)cipher_buf, plain_len) != HAL_OK) {
		CMD_ERR("3DES encrypt failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_cipher;
	}
	printf("ciphertest(hex):");
	cmd_print_uint8_array((uint8_t *)cipher_buf, cipher_len);

out_cipher:
    cmd_free(cipher_buf);
out_plain:
    cmd_free(plain_str);
    return ret;
}

static enum cmd_status cmd_decrypto_3des_exec(char *cmd)
{
	int32_t cnt;
	char mode_str[8];
	char key_str[25];
	char *cipher_str;
	uint8_t *plain_buf;
	uint8_t *cipher_buf;
	int8_t key_len = 0;
	uint32_t cipher_len = 0;
	CE_3DES_Config des_cfg;
    enum cmd_status ret = CMD_STATUS_ACKED;

    cipher_str = (char*)cmd_malloc(CE_CIPHER_MAX_LEN);
    if(cipher_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_cipher;
    }

	/* get param */
	cnt = cmd_sscanf(cmd, "m=%7s k=%s c=%s", mode_str, key_str, cipher_str);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	memset(&des_cfg, 0, sizeof(des_cfg));
	if (cmd_strcmp(mode_str, "ecb") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_ECB;
	} else if (cmd_strcmp(mode_str, "cbc") == 0) {
		des_cfg.mode = CE_CRYPT_MODE_CBC;
	} else {
		CMD_ERR("invalid mode %s\n", mode_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}

	key_len = cmd_strlen(key_str);
	if(24 != key_len) {
		CMD_ERR("invalid param:  key len %d != 24\n", key_len);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_cipher;
	}
	des_cfg.src = CE_CTL_KEYSOURCE_INPUT;
	memcpy(des_cfg.key, key_str, key_len);
	cipher_len = cmd_strlen(cipher_str);
	cipher_len = cipher_len >> 1;
	cipher_buf = (uint8_t*)cmd_malloc(cipher_len);
	cipher_str_to_arr(cipher_str, cipher_buf);
	plain_buf = (uint8_t*)cmd_malloc(cipher_len);
	if(HAL_3DES_Decrypt(&des_cfg, (uint8_t*)cipher_buf, (uint8_t*)plain_buf, cipher_len) != HAL_OK) {
		CMD_ERR("3DES encrypt failed\n");
		ret =  CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("plaintest:");
	cmd_raw_mode_enable();
	cmd_raw_mode_write(plain_buf, cipher_len);
	cmd_raw_mode_disable();
	printf("\n\n");

out_plain:
    cmd_free(cipher_buf);
    cmd_free(plain_buf);
out_cipher:
    cmd_free(cipher_str);
    return ret;
}

static enum cmd_status cmd_encrypto_crc_exec(char *cmd)
{
	char type_str[16];
	char* plain_str = NULL;
	int plain_len = 0;
	CE_CRC_Types type;
	CE_CRC_Handler crc;
	uint32_t res;
    int32_t cnt;

    enum cmd_status ret = CMD_STATUS_ACKED;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }
	/* get param */
	cnt = cmd_sscanf(cmd, "t=%16s p=%s", type_str, plain_str);
    /* check param */
    if (cnt != 2) {
        CMD_ERR("invalid param number %s\n", cmd);
        ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
    }

	/* check param */
	if (cmd_strcmp(type_str, "ibm") == 0) {
		type = CE_CRC16_IBM;
	} else if (cmd_strcmp(type_str, "maxim") == 0) {
		type = CE_CRC16_MAXIM;
	} else if (cmd_strcmp(type_str, "usb") == 0) {
		type = CE_CRC16_USB;
	} else if (cmd_strcmp(type_str, "modbus") == 0) {
		type = CE_CRC16_MODBUS;
	} else if (cmd_strcmp(type_str, "ccitt_1") == 0) {
		type = CE_CRC16_CCITT_1;
	} else if (cmd_strcmp(type_str, "ccitt") == 0) {
		type = CE_CRC16_CCITT;
	} else if (cmd_strcmp(type_str, "x25") == 0) {
		type = CE_CRC16_X25;
	} else if (cmd_strcmp(type_str, "xmodem") == 0) {
		type = CE_CRC16_XMODEM;
	} else if (cmd_strcmp(type_str, "dnp") == 0) {
		type = CE_CRC16_DNP;
	} else if (cmd_strcmp(type_str, "crc32") == 0) {
		type = CE_CRC32;
	} else if (cmd_strcmp(type_str, "mpeg2") == 0) {
		type = CE_CRC32_MPEG2;
	} else {
		CMD_ERR("invalid type %s\n", type_str);
		ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
	}
	plain_len = cmd_strlen(plain_str);
	if (HAL_CRC_Init(&crc, type, plain_len) != HAL_OK) {
		CMD_ERR("crc init failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	if ((HAL_CRC_Append(&crc, (uint8_t*)plain_str, plain_len) != HAL_OK)) {
		CMD_ERR("crc append failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	if (HAL_CRC_Finish(&crc, &res) != HAL_OK) {
		CMD_ERR("crc deinit failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("crc result:0x%x\n", res);

out_plain:
    cmd_free(plain_str);
    return ret;
}

static enum cmd_status cmd_encrypto_md5_exec(char *cmd)
{
	char* plain_str = NULL;
	int plain_len = 0;
	CE_MD5_Handler md5;
	uint32_t res[4];
    enum cmd_status ret = CMD_STATUS_ACKED;
    int32_t cnt;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }

	/* get param */
	cnt = cmd_sscanf(cmd, "p=%s", plain_str);
    /* check param */
    if (cnt != 1) {
        CMD_ERR("invalid param number %s\n", cmd);
        ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
    }

	if (HAL_MD5_Init(&md5, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		CMD_ERR("md5 init failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}

	plain_len = cmd_strlen(plain_str);
	if (HAL_MD5_Append(&md5, (uint8_t*)plain_str, plain_len) != HAL_OK) {
		CMD_ERR("md5 append failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	if (HAL_MD5_Finish(&md5, res) != HAL_OK) {
		CMD_ERR("md5 deinit failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("md5 result:");
	cmd_print_uint8_array((uint8_t*)res, 16);

out_plain:
    cmd_free(plain_str);
    return ret;
}

static enum cmd_status cmd_encrypto_sha1_exec(char *cmd)
{
	char* plain_str = NULL;
	int plain_len = 0;
	CE_SHA1_Handler sha1;
	uint32_t res[5];
    enum cmd_status ret = CMD_STATUS_ACKED;
    int32_t cnt;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }

	/* get param */
	cnt = cmd_sscanf(cmd, "p=%s", plain_str);

    /* check param */
    if (cnt != 1) {
        CMD_ERR("invalid param number %s\n", cmd);
        ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
    }

	if (HAL_SHA1_Init(&sha1, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		CMD_ERR("sha1 init failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}

	plain_len = cmd_strlen(plain_str);
	if (HAL_SHA1_Append(&sha1, (uint8_t*)plain_str, plain_len) != HAL_OK) {
		CMD_ERR("sha1 append failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	if (HAL_SHA1_Finish(&sha1, res) != HAL_OK) {
		CMD_ERR("sha1 deinit failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("sha1 result:");
	cmd_print_uint8_array((uint8_t*)res, 20);

out_plain:
    cmd_free(plain_str);
    return ret;
}

static enum cmd_status cmd_encrypto_sha256_exec(char *cmd)
{
	char* plain_str = NULL;
	int plain_len = 0;
	CE_SHA1_Handler sha1;
	uint32_t res[8];
    enum cmd_status ret = CMD_STATUS_ACKED;
    int32_t cnt;

    plain_str = (char*)cmd_malloc(CE_PLAIN_MAX_LEN);
    if(plain_str == NULL) {
        CMD_ERR("malloc failed\n");
        ret = CMD_STATUS_FAIL;
        goto out_plain;
    }
	/* get param */
	cnt = cmd_sscanf(cmd, "p=%s", plain_str);
    /* check param */
    if (cnt != 1) {
        CMD_ERR("invalid param number %s\n", cmd);
        ret = CMD_STATUS_INVALID_ARG;
        goto out_plain;
    }

	if (HAL_SHA256_Init(&sha1, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		CMD_ERR("sha1 init failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}

	plain_len = cmd_strlen(plain_str);
	if (HAL_SHA256_Append(&sha1, (uint8_t*)plain_str, plain_len) != HAL_OK) {
		CMD_ERR("sha256 append failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	if (HAL_SHA256_Finish(&sha1, res) != HAL_OK) {
		CMD_ERR("sha1 deinit failed\n");
		ret = CMD_STATUS_FAIL;
        goto out_plain;
	}
	printf("sha256 result:");
	cmd_print_uint8_array((uint8_t*)res, 32);

out_plain:
    cmd_free(plain_str);
    return ret;
}

static enum cmd_status cmd_random_prng_exec(char *cmd)
{
	uint8_t random[5];

	if (HAL_PRNG_Generate(random, 5) != HAL_OK) {
		CMD_ERR("prng generate failed\n");
		return CMD_STATUS_FAIL;
	}
	printf("prng result:");
	cmd_print_uint8_array(random, 5);

	return CMD_STATUS_ACKED;
}

#if (__CONFIG_CHIP_ARCH_VER == 2)
static enum cmd_status cmd_random_trng_exec(char *cmd)
{
	char type_str[3];
	uint8_t type;
	uint32_t ramdom[4];
    int32_t cnt;

	/* get param */
	cnt = cmd_sscanf(cmd, "t=%3s", type_str);
    /* check param */
    if (cnt != 1) {
        CMD_ERR("invalid param number %s\n", cmd);
        return CMD_STATUS_INVALID_ARG;
    }

	if(cmd_strcmp(type_str, "crc") == 0) {
		type = 0;
	} else if(cmd_strcmp(type_str, "xor") == 0) {
		type = 1;
	} else {
		return CMD_STATUS_FAIL;
	}

	if (HAL_TRNG_Extract(type, ramdom) != HAL_OK) {
		CMD_ERR("trng extract failed\n");
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	printf("trng result: %u %u %u %u\n", ramdom[0], ramdom[1], ramdom[2], ramdom[3]);

	return CMD_STATUS_ACKED;
}
#endif
static const struct cmd_data g_encrypto_cmds[] = {
	{ "aes", 	cmd_encrypto_aes_exec },
	{ "des",	cmd_encrypto_des_exec },
	{ "3des",	cmd_encrypto_3des_exec },
	{ "crc",	cmd_encrypto_crc_exec },
	{ "md5",	cmd_encrypto_md5_exec },
	{ "sha1",	cmd_encrypto_sha1_exec },
	{ "sha256",	cmd_encrypto_sha256_exec },
};

enum cmd_status cmd_encrypto_exec(char *cmd)
{
	return cmd_exec(cmd, g_encrypto_cmds, cmd_nitems(g_encrypto_cmds));
}

static const struct cmd_data g_decrypto_cmds[] = {
	{ "aes", 	cmd_decrypto_aes_exec },
	{ "des",	cmd_decrypto_des_exec },
	{ "3des",	cmd_decrypto_3des_exec },
};

enum cmd_status cmd_decrypto_exec(char *cmd)
{
	return cmd_exec(cmd, g_decrypto_cmds, cmd_nitems(g_decrypto_cmds));
}

static const struct cmd_data g_random_cmds[] = {
	{ "prng", 	cmd_random_prng_exec },
#if (__CONFIG_CHIP_ARCH_VER == 2)
	{ "trng",	cmd_random_trng_exec },
#endif
};

enum cmd_status cmd_random_exec(char *cmd)
{
	return cmd_exec(cmd, g_random_cmds, cmd_nitems(g_random_cmds));
}

static const struct cmd_data g_crypto_cmds[] = {
	{ "encrypto", 	cmd_encrypto_exec },
	{ "decrypto",	cmd_decrypto_exec },
	{ "random",		cmd_random_exec },
};

enum cmd_status cmd_crypto_exec(char *cmd)
{
	return cmd_exec(cmd, g_crypto_cmds, cmd_nitems(g_crypto_cmds));
}


