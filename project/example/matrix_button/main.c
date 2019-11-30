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

#include "common/framework/platform_init.h"
#include "common/apps/buttons/matrix_buttons.h"
#include "common/apps/buttons/matrix_buttons_low_level.h"
#include "stdio.h"

#define S1 R1C1
#define S2 R1C2
#define S3 R1C3
#define S4 R1C4
#define S5 R2C1

static matrix_button_handle *short_matrix_button;
static matrix_button_handle *long_matrix_button0;
static matrix_button_handle *long_matrix_button1;
static matrix_button_handle *short_long_matrix_button;
static matrix_button_handle *combined_long_matrix_button0;
static matrix_button_handle *repeat_long_matrix_button;

static void short_matrix_button_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* short button only has the RELEASE state */
	if (sta == RELEASE)
		printf("short button for S1 has release\n");
}

static void long_matrix_button0_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* long button has the PRESS/RELEASE state */
	if (sta == PRESS)
		printf("long button for S2 has press\n");
	else if (sta == RELEASE)
		printf("long button for S2 has release\n");
}

static void long_matrix_button1_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* long button has the PRESS/RELEASE state */
	if (sta == PRESS)
		printf("long button for S3 has press\n");
	else if (sta == RELEASE)
		printf("long button for S3 has release\n");
}

static void short_long_matrix_button_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* short_long button has the PRESS/RELEASE/REPEAT_RELEASE state */
	if (sta == PRESS)
		printf("long button for S4 has press\n");
	else if (sta == RELEASE)
		printf("short button for S4 has release\n");
	else if (sta == REPEAT_RELEASE)
		printf("long button for S4 has release\n");
}

static void combined_long_matrix_button0_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* combined button has the PRESS/RELEASE state */
	if (sta == PRESS)
		printf("combined button for S1|S2 has press\n");
	else if (sta == RELEASE)
		printf("combined button for S1|S2 has release\n");
}

static void repeat_long_matrix_button_cb(MATRIX_BUTTON_STATE sta, void *arg)
{
	/* repeat long button has the PRESS/RELEASE/REPEAT_PRESS state */
	if (sta == PRESS)
		printf("repeat long button for S5 has press\n");
	else if (sta == RELEASE)
		printf("repeat long button for S5 has release\n");
	else if (sta == REPEAT_PRESS)
		printf("repeat long button for S5 has repeat release\n");

}

int example_matrix_buttons_init(void)
{
	int ret;

	/* register the low level button interface */
	matrix_button_impl_t impl = {
		matrix_buttons_low_level_init,
		matrix_buttons_low_level_get_state,
		matrix_buttons_low_level_wait_semaphore,
		matrix_buttons_low_level_release_semaphore,
		matrix_buttons_low_level_deinit
	};
	/* set buttons thread priority and stack size */
	matrix_buttons_set_thread_priority(4);
	matrix_buttons_set_thread_stack_size(1024);

	/* init buttons, will init the low level buttons */
	ret = matrix_buttons_init(&impl);

	return ret;
}

void example_matrix_buttons_start(void)
{
	/* start buttons object */
	short_matrix_button->start(short_matrix_button);
	long_matrix_button0->start(long_matrix_button0);
	long_matrix_button1->start(long_matrix_button1);
	short_long_matrix_button->start(short_long_matrix_button);
	combined_long_matrix_button0->start(combined_long_matrix_button0);
	repeat_long_matrix_button->start(repeat_long_matrix_button);
}

void example_matrix_buttons_stop(void)
{
	/* stop buttons object */
	short_matrix_button->stop(short_matrix_button);
	long_matrix_button0->stop(long_matrix_button0);
	long_matrix_button1->stop(long_matrix_button1);
	short_long_matrix_button->stop(short_long_matrix_button);
	combined_long_matrix_button0->stop(combined_long_matrix_button0);
	repeat_long_matrix_button->stop(repeat_long_matrix_button);
}

void example_matrix_buttons_destroy(void)
{
	/* destory matrix buttons */
	if (short_matrix_button)
		short_matrix_button->destroy(short_matrix_button);
	if (long_matrix_button0)
		long_matrix_button0->destroy(long_matrix_button0);
	if (long_matrix_button1)
		long_matrix_button1->destroy(long_matrix_button1);
	if (short_long_matrix_button)
		short_long_matrix_button->destroy(short_long_matrix_button);
	if (combined_long_matrix_button0)
		combined_long_matrix_button0->destroy(combined_long_matrix_button0);
	if (repeat_long_matrix_button)
		repeat_long_matrix_button->destroy(repeat_long_matrix_button);
}

int example_matrix_buttons_create(void)
{
	/* create matrix buttons object */

	/* when this matrix button pressed, it will trigger */
	short_matrix_button = create_short_matrix_button(S1);
	/* after a long press of 50ms, it will trigger */
	long_matrix_button0 = create_long_matrix_button(S2, 50);
	/* after a long press of 500ms, it will trigger */
	long_matrix_button1 = create_long_matrix_button(S3, 500);

	short_long_matrix_button = create_short_long_matrix_button(S4, 500);

	combined_long_matrix_button0 = create_combined_long_matrix_button(S1 | S2, 50);

	repeat_long_matrix_button = create_repeat_long_matrix_button(S5, 500, 300);

	if (!short_matrix_button || !long_matrix_button0 || !long_matrix_button1 || !short_long_matrix_button ||
		!combined_long_matrix_button0 || !repeat_long_matrix_button) {
			printf("at least one matrix button create err\n");
			goto exit;
	}

	/* set matrix buttons callback */
	short_matrix_button->cb = short_matrix_button_cb;
	long_matrix_button0->cb = long_matrix_button0_cb;
	long_matrix_button1->cb = long_matrix_button1_cb;
	short_long_matrix_button->cb = short_long_matrix_button_cb;
	combined_long_matrix_button0->cb = combined_long_matrix_button0_cb;
	repeat_long_matrix_button->cb = repeat_long_matrix_button_cb;

	return 0;

exit:
	example_matrix_buttons_destroy();
	return -1;
}


int main(void)
{
	platform_init();

	example_matrix_buttons_init();
	example_matrix_buttons_create();
	example_matrix_buttons_start();

	return 0;
}
