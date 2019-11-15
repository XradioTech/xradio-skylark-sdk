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
#include "util/time_logger.h"

#define SAVE_TIME_TAB		0x00200F80
#define MAX_SIZE_OF_TIME_ITEM	3
char * time_item[MAX_SIZE_OF_TIME_ITEM] = {
//"begin boot",//0
"begin app",
"end pf init",
//"send auth",
//"send assoc",//4
//"net if up",
"end connection",//6
};

void save_time(uint32_t t, uint32_t i)
{
	((uint32_t *)SAVE_TIME_TAB)[i] = t;
}

void get_time(void)
{
	printf("\n");
	for(int i = 0;i < MAX_SIZE_OF_TIME_ITEM;i++) {
		if (i == 0)
			printf("%s:\t%06dus\tdelta:%dus\n", time_item[i], ((uint32_t *)SAVE_TIME_TAB)[i],
										((uint32_t *)SAVE_TIME_TAB)[i]);
		else
			printf("%s:\t%06dus\tdelta:%dus\n", time_item[i], ((uint32_t *)SAVE_TIME_TAB)[i],
										((uint32_t *)SAVE_TIME_TAB)[i] - ((uint32_t *)SAVE_TIME_TAB)[i-1]);
	}
	printf("delta of %s to %s:\t%dus\n", time_item[0], time_item[MAX_SIZE_OF_TIME_ITEM - 1],
				((uint32_t *)SAVE_TIME_TAB)[MAX_SIZE_OF_TIME_ITEM - 1] - ((uint32_t *)SAVE_TIME_TAB)[0]);
}

