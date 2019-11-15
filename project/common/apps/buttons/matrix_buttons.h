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

#ifndef _APPS_MATRIX_BUTTONS_H_
#define _APPS_MATRIX_BUTTONS_H_

#include <stdint.h>
#include "matrix_buttons_id.h"

#define INVALID_MATRIX_BUTTON_STATE 0xffffffff
#define ALL_MATRIX_BUTTONS_RELEASE  0

typedef enum {
	SHORT_BUTTON,
	LONG_BUTTON,
	SHORT_LONG_BUTTON,
	COMBINED_LONG_BUTTON,
	REPEAT_LONG_BUTTON,
} MATRIX_BUTTON_TYPE;

typedef enum {
	PRESS,			/* used in LONG_BUTTON/SHORT_LONG_BUTTON/COMBINED_LONG_BUTTON/REPEAT_LONG_BUTTON */
	RELEASE,        /* used in all types of buttons */
	REPEAT_PRESS,   /* used in REPEAT_LONG_BUTTON */
	REPEAT_RELEASE, /* used in SHORT_LONG_BUTTON */
	INVALID_STA,    /* invalid button state */
} MATRIX_BUTTON_STATE;

typedef struct matrix_button_handle {
	void (*start)(struct matrix_button_handle *handle);
	void (*stop)(struct matrix_button_handle *handle);
	int (*destroy)(struct matrix_button_handle *handle);
	int (*get_state)(struct matrix_button_handle *handle);
	void (*cb)(MATRIX_BUTTON_STATE sta, void *arg); /* matrix_buttons callback function */
	void *arg; /* the argument to callback function */
} matrix_button_handle;

typedef struct {
	int (*low_level_init)(void);
	int (*low_level_get_state)(void);
	int (*low_level_wait_semaphore)(uint32_t waitms);
	int (*low_level_release_semaphore)(void);
	int (*low_level_deinit)(void);
} matrix_button_impl_t;

int matrix_buttons_init(matrix_button_impl_t* impl);
int matrix_buttons_deinit(void);
void matrix_buttons_set_thread_stack_size(uint32_t size);
void matrix_buttons_set_thread_priority(uint32_t priority);

matrix_button_handle* create_short_matrix_button(uint32_t id_mask);
matrix_button_handle* create_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms);
matrix_button_handle* create_short_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms);
matrix_button_handle* create_combined_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms);
matrix_button_handle* create_repeat_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms, uint32_t repeat_timeout_ms);

#endif /* _APPS_MATRIX_BUTTONS_H_ */


