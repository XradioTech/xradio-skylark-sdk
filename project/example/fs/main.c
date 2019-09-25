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

#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"
#include "fs/fatfs/ff.h"
#include "common/framework/platform_init.h"
#include "common/framework/fs_ctrl.h"

#define FS_CARD_INFO_TEST	1
#define FS_FILE_PRINT_TEST	0
#define FS_FILE_COPY_TEST	1
#define FS_FILE_SCAN_TEST	1
#define FS_DIR_RM_TEST		1
#define FS_DIR_COPY_TEST	1

#define PRINT_LOG(INFO)		printf("\n/**********************************\n");	\
							printf(" * %s\n", INFO);							\
							printf(" **********************************/\n");

static int fs_init()
{
	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
		return -1;
	}

	printf("mount success\n");

	return 0;
}

/* unit MB */
int fs_get_card_info(uint32_t *free_capacity, uint32_t *total_capacity)
{
	DWORD nclst;
	FATFS* fs;

	if (f_getfree("0:/", &nclst, &fs) == FR_OK) {
		uint32_t sector_size;
#if (_MAX_SS == _MIN_SS)
		sector_size = _MAX_SS;
#else
		sector_size = fs->ssize;
#endif
		*free_capacity = (float)fs->free_clst*fs->csize/1024/1024*sector_size;
		*total_capacity = (float)(fs->n_fatent-2)*fs->csize/1024/1024*sector_size;
	} else {
		printf("get card info fail");
		return -1;
	}

	return 0;
}

int fs_scan_files(char* path)
{
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = fs_scan_files(path);                    /* Enter the directory */
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                printf("%s/%s\n", path, fno.fname);
            }
        }
        f_closedir(&dir);
    }

    return res;
}

int fs_copy_file(const char *file_dst, const char *file_src)
{
	FRESULT res;
	UINT br, bw;		 /* File read/write count */
	FIL fsrc, fdst;

	res = f_open(&fsrc, file_src, FA_READ);
    if (res != FR_OK) {
		printf("open file fail, %d\n", res);
		return res;
	}

    res = f_open(&fdst, file_dst, FA_WRITE | FA_CREATE_ALWAYS);
	if (res != FR_OK) {
		printf("open file fail, %d\n", res);
		f_close(&fsrc);
		return res;
	}

	char *buf = malloc(1024); /* Line buffer */
	memset(buf, 0, sizeof(*buf));

    /* Copy source to destination */
    for (; ;) {
        res = f_read(&fsrc, buf, sizeof(buf), &br);	/* Read a chunk of source file */
        if (res || br == 0)
			break; /* error or eof */
        res = f_write(&fdst, buf, br, &bw);	/* Write it to the destination file */
        if (res || bw < br) break; /* error or disk full */
    }

    /* Close open files */
    f_close(&fsrc);
    f_close(&fdst);

	free(buf);

	return res;
}

int fs_rm_dir(char *dir_path)
{
	FRESULT res;
    DIR dir;
	UINT i;
    static FILINFO fno;

    res = f_opendir(&dir, dir_path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */

            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
				i = strlen(dir_path);
				sprintf(&dir_path[i], "/%s", fno.fname);
                res = fs_rm_dir(dir_path);                 /* Enter the directory */
                if (res != FR_OK) break;
				dir_path[i] = 0;
            } else {                                       /* It is a file. */
            	static char file_path[256];
				sprintf(file_path, "%s/%s", dir_path, fno.fname);
				res = f_unlink(file_path);
				printf("delete file %s %s\n", file_path, (res != FR_OK) ? "failed" : "success");
				if (res != FR_OK) {
					break;
				}
            }
        }
        f_closedir(&dir);

		if (res == FR_OK) {
			res = f_unlink(dir_path);
			printf("delete dir %s %s\n", dir_path, (res != FR_OK) ? "failed" : "success");
		}
    }

	return res;
}

int fs_copy_dir(char *dir_dst, char *dir_src)
{
	FRESULT res;
	DIR dir;
	UINT i, j;
	static FILINFO fno;

	res = f_opendir(&dir, dir_src);					   /* Open the directory */
	if (res == FR_OK) {
		res = f_mkdir(dir_dst);
		if (res != FR_OK && res != FR_EXIST) {
			printf("create dir %s failed, %d\n", dir_dst, res);
			return res;
		}

		for (;;) {
			res = f_readdir(&dir, &fno);				   /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */

			if (fno.fattrib & AM_DIR) { 				   /* It is a directory */
				sprintf(dir_src, "%s/%s", dir_src, fno.fname);
				sprintf(dir_dst, "%s/%s", dir_dst, fno.fname);
				res = fs_copy_dir(dir_dst, dir_src);		/* Enter the directory */
				if (res != FR_OK) break;
			} else {									   /* It is a file. */
				i = strlen(dir_src);
				sprintf(&dir_src[i], "/%s", fno.fname);
				j = strlen(dir_dst);
				sprintf(&dir_dst[j], "/%s", fno.fname);

				res = fs_copy_file(dir_dst, dir_src);
				printf("copy file %s %s\n", dir_dst, (res != FR_OK) ? "failed" : "success");
				if (res != FR_OK) {
					break;
				}
				dir_src[i] = 0;
				dir_dst[j] = 0;
			}
		}
		f_closedir(&dir);
	}

	return res;
}

#if FS_CARD_INFO_TEST
static void fs_card_info_test()
{
	PRINT_LOG("FS Card Info Test..");

	uint32_t free_capacity, total_capacity;
	if (fs_get_card_info(&free_capacity, &total_capacity) == 0) {
		printf("card info: free capacity=%.2fG, total capacity=%.2fG\n",
			(uint32_t)((float)free_capacity/1024*100)/100.0f,
			(uint32_t)((float)total_capacity/1024*100)/100.0f);
	}
}
#endif

#if FS_FILE_PRINT_TEST
static int fs_print_file_test()
{
	PRINT_LOG("FS File Print Test..");

	FIL fp;        /* File object */
	FRESULT res;     /* FatFs return code */

	char *buf = malloc(100); /* Line buffer */
	if (!buf) {
		printf("malloc fail\n");
		return -1;
	}

    /* Open a text file */
    res = f_open(&fp, "print_file.txt", FA_READ);
	if (res != FR_OK) {
		printf("open fail, %d\n", res);
		free(buf);
		return -1;
	}

    /* Read every line and display it */
    while (f_gets(buf, sizeof(buf), &fp)) {
        printf(buf);
    }
	printf("\n");

    /* Close the file */
    f_close(&fp);

	free(buf);

    return 0;
}
#endif

#if FS_FILE_COPY_TEST
static void fs_copy_file_test()
{
	PRINT_LOG("FS File Copy Test..");

	char *file_src = "copy_file.txt";
	char *file_dst = "copy_file(1).txt";

	if (fs_copy_file(file_dst, file_src) == 0)
		printf("copy file success\n");
	else
		printf("copy file fail\n");
}
#endif

#if FS_FILE_SCAN_TEST
static void fs_scan_file_test()
{
	PRINT_LOG("FS File Scan Test..");

	char path[256];
    strcpy(path, "/");
	fs_scan_files(path);
}
#endif

#if FS_DIR_RM_TEST
static void fs_rm_dir_test()
{
	PRINT_LOG("FS DIR Detele Test..");

	char dir_path[256];
	strcpy(dir_path, "dir_test");
	fs_rm_dir(dir_path);
}
#endif

#if FS_DIR_COPY_TEST
static void fs_copy_dir_test()
{
	PRINT_LOG("FS DIR Copy Test..");

	char dir_dst[256];
	char dir_src[256];
	strcpy(dir_dst, "dir_dst");
	strcpy(dir_src, "dir_src");

	fs_copy_dir(dir_dst, dir_src);
}

#endif

static int fs_deinit()
{
	if (fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("unmount fail\n");
		return -1;
	}

	printf("\nunmount success\n");

	return 0;
}

/* Run this demo, please connect the XR872ET_VER_DIG_V1_0 board. */
int main(void)
{
	platform_init();

	printf("fs demo started\n\n");

	if (fs_init() != 0)
		return -1;

#if FS_CARD_INFO_TEST
	fs_card_info_test();
#endif

#if FS_FILE_PRINT_TEST
	fs_print_file_test();
#endif

#if FS_FILE_COPY_TEST
	fs_copy_file_test();
#endif

#if FS_FILE_SCAN_TEST
	fs_scan_file_test();
#endif

#if FS_DIR_RM_TEST
	fs_rm_dir_test();
#endif

#if FS_DIR_COPY_TEST
	fs_copy_dir_test();
#endif

	fs_deinit();

	printf("fs demo over\n");

	return 0;
}

