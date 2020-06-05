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

#ifdef __CONFIG_XPLAYER

#include <stdlib.h>
#include "fs/fatfs/ff.h"
#include "cedarx_fs.h"
#include "cdx_log.h"
#include <string.h>

//#define TEST_TS

#define DBG(fmt, ...)   do{ printf("[Cedarx OS porting DBG] %s line %d, "fmt, __func__, __LINE__, ##__VA_ARGS__); } while(0)
#define BUG()           do{ DBG("BUG happend!\n"); }while(0)

#define ALERT(fmt, ...) printf("[Cedarx OS porting ALERT] <%s> " fmt, __func__, ##__VA_ARGS__)

#define ENTRY()
#define INVALID()


FILE * cedarx_fopen(const char * filename, const char * mode) // i think this would not be used.
{
    unsigned char fmode = 0;

//  DBG("filename - %s.\n", filename);
    FILE * file = malloc(sizeof(FILE));
    if (file == NULL)
        goto out;

    if (strstr(mode, "a"))
        fmode |= FA_OPEN_APPEND;
    if (strstr(mode, "+"))
        fmode |= FA_READ | FA_WRITE;
    if (strstr(mode, "w"))
        fmode |= FA_WRITE | FA_CREATE_ALWAYS;
    if (strstr(mode, "r"))
        fmode |= FA_READ | FA_OPEN_EXISTING;

    FRESULT res = f_open(file, filename, fmode);
    if (res != FR_OK) {
        DBG("open file\"%s\" failed: %d\n", filename, res);
        free(file);
        file = NULL;
    }

out:
    return file;
}

int cedarx_fread(void * ptr, int size, int nmemb, FILE * stream)
{
    unsigned int ret;
    FRESULT res;

    if (stream == NULL)
        return -1;

    res = f_read(stream, ptr, size * nmemb, &ret);
    if (res != FR_OK) {
        DBG("read file failed: %d\n", res);
        return (int)-res;
    }

    return (int)(ret / size);
}

int cedarx_fwrite(const void * ptr, int size, int nmemb, FILE * stream)
{
    unsigned int ret;
    FRESULT res;

    if (stream == NULL)
        return -1;

    res = f_write(stream, ptr, size * nmemb, &ret);
    if (res != FR_OK) {
        DBG("read file failed: %d\n", res);
        return (int)-res;
    }

    return (int)(ret / size);
}

int cedarx_fseek(FILE * stream, long long offset, int whence)
{
    FRESULT res;
    long long foffset;

    if (stream == NULL)
        return -1;

    if (whence == SEEK_SET)
        foffset = offset;
    else if (whence == SEEK_CUR)
        foffset = f_tell(stream) + offset;
    else if (whence == SEEK_END)
        foffset = f_size(stream) + offset;
    else
        return -1;

    res = f_lseek(stream, foffset);
    if (res != FR_OK)
        DBG("read file failed: %d\n", res);

    return (int)-res;
}

long long cedarx_ftell(FILE * stream)
{
    if (stream == NULL)
        return -1;

    return f_tell(stream);
}

int cedarx_fclose(FILE * stream)
{
    FRESULT res;

    if (stream == NULL)
        return -1;

    res = f_close(stream);
    if (res == FR_OK)
        free(stream);

    return (int)-res;
}
#endif
