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

#ifndef _FS_
#define _FS_

#include "cdx_log.h"
#include "fs/fatfs/ff.h"

//#define O_RDONLY        00000000
//#define O_WRONLY        00000001
//#define O_RDWR          00000002
//#define O_CREAT         00000100  /* not fcntl */
#ifdef __cplusplus
extern "C"
{
#endif

#define FILE FIL

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

FILE * cedarx_fopen(const char * filename, const char * mode);
int cedarx_fseek(FILE * stream, long long offset, int whence);
long long cedarx_ftell(FILE * stream);
int cedarx_fclose(FILE * stream);
int cedarx_fwrite(const void * ptr, int size, int nmemb, FILE * stream);
int cedarx_fread(void * ptr, int size, int nmemb, FILE * stream);

#define fopen cedarx_fopen
#define fseek cedarx_fseek
#define ftell cedarx_ftell
#define fclose cedarx_fclose
#define fwrite cedarx_fwrite
#define fread cedarx_fread

#ifdef __cplusplus
}
#endif

#endif
