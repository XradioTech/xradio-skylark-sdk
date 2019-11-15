/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions
 *	are met:
 *	  1. Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	  2. Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in the
 *		 documentation and/or other materials provided with the
 *		 distribution.
 *	  3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *		 its contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "sys/defs.h"
#include "matrix_buttons.h"
#include "kernel/os/os.h"
#include "sys/list.h"

typedef struct {
	uint32_t id_mask;
	uint32_t repeat_timeout_ms; /* only used in repeat matrix button */
	uint32_t timeout_ms;        /* the timeout to trigger the matrix button object */
	uint8_t en;                 /* enable/disable the matrix button object */
	MATRIX_BUTTON_TYPE func;           /* the matrix button type */
	MATRIX_BUTTON_STATE state;         /* the matrix button state */
	matrix_button_handle handle;       /* the matrix button handle */
	struct list_head node;      /* all the matrix_buttons object will be added in one matrix button list */
} matrix_button_obj;

#define MATRIX_BUTTON_DBG  0

#if MATRIX_BUTTON_DBG
#define MATRIX_BUTTON_DEBUG(msg, arg...)      printf("[matrix button debug] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#define MATRIX_BUTTON_WARNING(msg, arg...)    printf("[matrix button warning] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#else
#define MATRIX_BUTTON_DEBUG(msg, arg...)
#define MATRIX_BUTTON_WARNING(msg, arg...)
#endif
#define MATRIX_BUTTON_ERR(msg, arg...)        printf("[matrix button err] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)

static matrix_button_impl_t matrix_button_impl;
static struct list_head matrix_buttons_head;
static bool matrix_buttons_timer_time_up = 0;
static OS_Timer_t matrix_buttons_timer;
static OS_Thread_t matrix_buttons_thread;
static bool matrix_buttons_thread_run_flag = 0;
static uint32_t matrix_buttons_thread_stack_size = 0;
static OS_Priority matrix_buttons_thread_priority = OS_PRIORITY_NORMAL;
static matrix_button_obj *current_tiggered_obj = NULL;

#define MATRIX_BUTTONS_THREAD_STACK_SIZE_DEFAULT 512
#define GET_BASE(prt) (matrix_button_obj *)container_of(prt, matrix_button_obj, handle)

static void short_matrix_button_release(int matrix_buttons_state)
{
	/* a matrix button can be released only when the matrix button state is all 0,
	 * in other cases, it is determined that it is interrupted in midway. Only when the
	 * matrix button has been released, the callback function can be called.
	 */
	if (matrix_buttons_state == ALL_MATRIX_BUTTONS_RELEASE) {
		if (current_tiggered_obj->state == PRESS) {
			current_tiggered_obj->state = RELEASE;
			if (current_tiggered_obj->handle.cb)
				current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
		}
	}

	/* Whether it is a complete release or a middle interruption,
	 * the matrix button state must set to an invalid state.
	 */
	current_tiggered_obj->state = INVALID_STA;
	/* the pointer can only be reset when the matrix_buttons are fully released. */
	if (matrix_buttons_state == ALL_MATRIX_BUTTONS_RELEASE)
		current_tiggered_obj = NULL;
}

static void long_matrix_button_release(int matrix_buttons_state)
{
	/* whether the timer is active or not, stop the timer and reset the flag. */
	OS_TimerStop(&matrix_buttons_timer);
	matrix_buttons_timer_time_up = false;

	if (current_tiggered_obj->state == PRESS || current_tiggered_obj->state == REPEAT_PRESS) {
		current_tiggered_obj->state = RELEASE;
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
	}

	current_tiggered_obj->state = INVALID_STA;
	if (matrix_buttons_state == ALL_MATRIX_BUTTONS_RELEASE)
		current_tiggered_obj = NULL;
}

static void short_long_matrix_button_release(int matrix_buttons_state)
{
	/* if the timer is active, that mean press_time < timeout_ms. */
	if (OS_TimerIsActive(&matrix_buttons_timer)) {
		/* stop the timer and set the RELEASE state, call the callback function. */
		OS_TimerStop(&matrix_buttons_timer);
		matrix_buttons_timer_time_up = false;
		current_tiggered_obj->state = RELEASE;
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
	}
	/* else the timer is not active, that mean press_time >= timeout_ms. */
	else if (current_tiggered_obj->state == PRESS) {
		current_tiggered_obj->state = REPEAT_RELEASE;
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
	}

	current_tiggered_obj->state = INVALID_STA;
	if (matrix_buttons_state == ALL_MATRIX_BUTTONS_RELEASE)
		current_tiggered_obj = NULL;
}

static void repeat_long_matrix_button_release(int matrix_buttons_state)
{
	long_matrix_button_release(matrix_buttons_state);
}

static void combined_long_matrix_button_release(int matrix_buttons_state)
{
	OS_TimerStop(&matrix_buttons_timer);
	matrix_buttons_timer_time_up = false;

	if (current_tiggered_obj->state == PRESS) {
		current_tiggered_obj->state = RELEASE;
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
	}

	current_tiggered_obj->state = INVALID_STA;
	if (matrix_buttons_state == ALL_MATRIX_BUTTONS_RELEASE)
		current_tiggered_obj = NULL;
}

static void short_matrix_button_press(matrix_button_obj *obj)
{
	/* if not NULL, that mean the previous matrix button has not been fully released. */
	if (current_tiggered_obj)
		return;

	/* set PRESS state of short matrix button */
	obj->state = PRESS;
	/* record the current triggered matrix button object */
	current_tiggered_obj = obj;
}

static void long_matrix_button_press(matrix_button_obj *obj)
{
	OS_Status sta;

	/* if not NULL, that mean the previous matrix button has not been fully released. */
	if (current_tiggered_obj)
		return;

	/* set the time */
	sta = OS_TimerChangePeriod(&matrix_buttons_timer, obj->timeout_ms);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix button timer change period error");
		return;
	}
	sta = OS_TimerStart(&matrix_buttons_timer);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix button timer start error");
		return;
	}

	/* matrix button has not been triggerd, the state should be INVALID_STA. */
	obj->state = INVALID_STA;
	/* record the current triggered matrix button object */
	current_tiggered_obj = obj;
}

static void short_long_matrix_button_press(matrix_button_obj *obj)
{
	long_matrix_button_press(obj);
}

static void combined_long_matrix_button_press(matrix_button_obj *obj)
{
	OS_Status sta;
	int matrix_buttons_state = obj->id_mask;

	/* The combination long matrix button is a subset of long press matrix_buttons.
	 * Because all the matrix_buttons can not be pressed at the same time,
	 * so the combination matrix button can interrupt long press buttton and short press matrix button.
	 */
	/* if one matrix button object has been triggered, then release it, interrupt it. */
	if (current_tiggered_obj && current_tiggered_obj->func == SHORT_BUTTON)
		short_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj && current_tiggered_obj->func == LONG_BUTTON)
		long_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj && current_tiggered_obj->func == SHORT_LONG_BUTTON)
		short_long_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj && current_tiggered_obj->func == COMBINED_LONG_BUTTON) {
		combined_long_matrix_button_release(matrix_buttons_state);
		/* combination matrix button can not interrupt combination matrix button,
		 * and the combination matrix button must wait all the matrix_buttons released.
		 */
		return;
	}

	/* set the time */
	sta = OS_TimerChangePeriod(&matrix_buttons_timer, obj->timeout_ms);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix button timer change period error");
		return;
	}
	sta = OS_TimerStart(&matrix_buttons_timer);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix button timer start error");
		return;
	}

	/* matrix button has not been triggerd, the state should be INVALID_STA. */
	obj->state = INVALID_STA;
	/* record the current triggered matrix button object */
	current_tiggered_obj = obj;
}

static void repeat_long_matrix_button_press(matrix_button_obj *obj)
{
	long_matrix_button_press(obj);
}

static void matrix_button_timer_event(void)
{
	OS_Status sta;

	if (!current_tiggered_obj)
		return;

	/* if current matrix button is LONG_BUTTON/SHORT_LONG_BUTTON/COMBINED_LONG_BUTTON,
	 * matrix button timer time's up mean that the press time long enough than timeout_ms.
	 * so the matrix button state should be set PRESS.
	 */
	if (current_tiggered_obj->func == LONG_BUTTON ||
		current_tiggered_obj->func == SHORT_LONG_BUTTON ||
		current_tiggered_obj->func == COMBINED_LONG_BUTTON) {
		/* set the state */
		current_tiggered_obj->state = PRESS;
		/* call the callback function */
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
	}
	/* if the matrix button is REPEAT_LONG_BUTTON */
	else if (current_tiggered_obj->func == REPEAT_LONG_BUTTON) {
		/* if the matrix button state is INVALID_STA, that mean it is the first time trigger the matrix button. */
		if (current_tiggered_obj->state == INVALID_STA)
			/* set the PRESS state */
			current_tiggered_obj->state = PRESS;
		/* if the state is PRESS/REPEAT_PRESS, that mean it is not the first time trigger the matrix button. */
		else if (current_tiggered_obj->state == PRESS || current_tiggered_obj->state == REPEAT_PRESS)
			/* set the REPEAT_PRESS state */
			current_tiggered_obj->state = REPEAT_PRESS;

		/* call the callback function */
		if (current_tiggered_obj->handle.cb)
			current_tiggered_obj->handle.cb(current_tiggered_obj->state, current_tiggered_obj->handle.arg);
		/* whether it is the first time trigger the matrix button or not, the timer should be changed. */
		sta = OS_TimerChangePeriod(&matrix_buttons_timer, current_tiggered_obj->repeat_timeout_ms);
		if (sta != OS_OK) {
			MATRIX_BUTTON_ERR("matrix button timer change period error");
			return;
		}
		sta = OS_TimerStart(&matrix_buttons_timer);
		if (sta != OS_OK) {
			MATRIX_BUTTON_ERR("matrix button timer start error");
			return;
		}
	}
}

static void release_current_tiggered_obj(int matrix_buttons_state)
{
	if (!current_tiggered_obj)
		return;

	if (current_tiggered_obj->func == SHORT_BUTTON)
		short_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj->func == LONG_BUTTON)
		long_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj->func == SHORT_LONG_BUTTON)
		short_long_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj->func == COMBINED_LONG_BUTTON)
		combined_long_matrix_button_release(matrix_buttons_state);
	else if (current_tiggered_obj->func == REPEAT_LONG_BUTTON)
		repeat_long_matrix_button_release(matrix_buttons_state);
}

static matrix_button_obj* get_matrix_buttons_obj(uint32_t id_mask)
{
	matrix_button_obj *obj;

	if (list_empty(&matrix_buttons_head))
		return NULL;

	list_for_each_entry(obj, &matrix_buttons_head, node)
		if (obj->id_mask == id_mask)
			return obj;

	return NULL;
}

static void matrix_buttons_process_thread(void *arg)
{
	int matrix_buttons_state = 0;
	int prev_matrix_buttons_state = 0;
	matrix_button_obj *obj;

	while (matrix_buttons_thread_run_flag) {
		/* wait a matrix button trigger */
		if (matrix_button_impl.low_level_wait_semaphore)
			matrix_button_impl.low_level_wait_semaphore(OS_WAIT_FOREVER);

		/* cycle scan matrix button status */
		while (1) {
			/* if matrix button timer time is up, then process the timer event. */
			if (matrix_buttons_timer_time_up) {
				matrix_button_timer_event();
				matrix_buttons_timer_time_up = false;
				goto contin;
			}

			/* get all the matrix_buttons' state form low levle matrix_buttons */
			if (matrix_button_impl.low_level_get_state)
				matrix_buttons_state = matrix_button_impl.low_level_get_state();
			else
				matrix_buttons_state = 0;

			/* if the matrix_buttons state not change, then continue
			 * sometimes the AD matrix_buttons will trigger by mistake.
			 */
			if (prev_matrix_buttons_state == matrix_buttons_state)
				goto contin;
			prev_matrix_buttons_state = matrix_buttons_state;
#if MATRIX_BUTTON_DBG
			char s[33] = {0};
			int n = matrix_buttons_state;
			for (int i = 31; i >= 0; i--)
				s[31-i] = (n&(1<<i)) == 0 ? '0' : '1';
			MATRIX_BUTTON_DEBUG("row:%.*s col:%.*s", 16, s, 16, s + 16);
#endif
			/* get matrix button's object based on state */
			obj = get_matrix_buttons_obj(matrix_buttons_state);

			/* if not NULL, that mean one matrix button object has been pressed,
			 * must release it or let it in invalid state first.
			 */
			if (current_tiggered_obj)
				release_current_tiggered_obj(matrix_buttons_state);

			/* if find one matrix button object, then press and record it. */
			if (obj) {
				if(!obj->en)
					goto contin;
				if (obj->func == SHORT_BUTTON)
					short_matrix_button_press(obj);
				else if (obj->func == LONG_BUTTON)
					long_matrix_button_press(obj);
				else if (obj->func == SHORT_LONG_BUTTON)
					short_long_matrix_button_press(obj);
				else if (obj->func == COMBINED_LONG_BUTTON)
					combined_long_matrix_button_press(obj);
				else if (obj->func == REPEAT_LONG_BUTTON)
					repeat_long_matrix_button_press(obj);
			}
contin:
			if (matrix_buttons_state != ALL_MATRIX_BUTTONS_RELEASE)
				OS_MSleep(10);
			else
				break;
		}

	}

	OS_ThreadDelete(&matrix_buttons_thread);
}

static void matrix_buttons_timer_cb(void *arg)
{
	/* release the semaphore, let the matrix_buttons thread running. */
	if (matrix_button_impl.low_level_release_semaphore)
		matrix_button_impl.low_level_release_semaphore();
	matrix_buttons_timer_time_up = true;
}

/**
  * @brief Set the stack size of the matrix_buttons thread.
  * @note If the callback function of matrix_buttons need lots of stacks, then need
  *       add the matrix_buttons thread's stack.
  * @param priority: The stack size of matrix_buttons thread.
  * @retval void.
  */
void matrix_buttons_set_thread_stack_size(uint32_t size)
{
	if (size > 0)
		matrix_buttons_thread_stack_size = size;
}

/**
  * @brief Set the priority of the matrix_buttons thread.
  * @note none.
  * @param priority: The priority of matrix_buttons thread.
  * @retval void.
  */
void matrix_buttons_set_thread_priority(uint32_t priority)
{
	if (priority >= OS_PRIORITY_IDLE && priority <= OS_PRIORITY_REAL_TIME)
		matrix_buttons_thread_priority = priority;
}

/**
  * @brief Buttons initialize.
  * @note Initialize the matrix button module.
  * @param impl: The interface that the low level matrix_buttons pass to the matrix_buttons module,
  *              which is used to operate the low level matrix_buttons.
  * @retval 0: success, -1: fail
  */
int matrix_buttons_init(matrix_button_impl_t* impl)
{
	int ret;
	OS_Status sta;

	if (!impl) {
		MATRIX_BUTTON_ERR("matrix_buttons impl is NULL");
		return -1;
	}
	memcpy(&matrix_button_impl, impl, sizeof(matrix_button_impl));

	/* initialize the low level matrix_buttons */
	if (matrix_button_impl.low_level_init) {
		ret = matrix_button_impl.low_level_init();
		if (ret != 0) {
			MATRIX_BUTTON_ERR("matrix_buttons low level init err");
			return -1;
		}
	}

	matrix_buttons_thread_run_flag = 1;
	sta = OS_ThreadCreate(&matrix_buttons_thread,
						  "matrix_buttons_thread",
                          matrix_buttons_process_thread,
                          NULL,
                          matrix_buttons_thread_priority != OS_PRIORITY_NORMAL ?
                          matrix_buttons_thread_priority : OS_PRIORITY_NORMAL,
                          matrix_buttons_thread_stack_size > 0 ?
                          matrix_buttons_thread_stack_size : MATRIX_BUTTONS_THREAD_STACK_SIZE_DEFAULT);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix_buttons thread create error");
		return -1;
	}

	/* create timer for matrix_buttons */
	sta = OS_TimerCreate(&matrix_buttons_timer, OS_TIMER_ONCE, matrix_buttons_timer_cb, NULL, OS_WAIT_FOREVER);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix_buttons timer create error");
		return -1;
	}

	INIT_LIST_HEAD(&matrix_buttons_head);

	return 0;
}

/**
  * @brief Buttons deinitialize.
  * @note Deinitialize the matrix_buttons module.
  *       The interface will delete and free all the matrix_buttons objects.
  * @param void
  * @retval 0: success, -1: fail
  */
int matrix_buttons_deinit(void)
{
	matrix_button_obj *obj_t;

	if (list_empty(&matrix_buttons_head))
		return 0;

	matrix_buttons_thread_run_flag = 0;
	/* release the semaphore,
	 * prevent the matrix_buttons thread from waiting for the semaphore all the time.
	 */
	if (matrix_button_impl.low_level_release_semaphore)
		matrix_button_impl.low_level_release_semaphore();

	/* waiting for the matrix_buttons thread delete */
	while (OS_ThreadIsValid(&matrix_buttons_thread))
		OS_MSleep(1);

	/* delete and free all the matrix button objects */
	while (!list_empty(&matrix_buttons_head)) {
		obj_t = list_first_entry(&matrix_buttons_head, matrix_button_obj, node);
		list_del(&obj_t->node);
		free(obj_t);
	}

	if (OS_TimerIsValid(&matrix_buttons_timer))
		OS_TimerDelete(&matrix_buttons_timer);

	if (matrix_button_impl.low_level_deinit)
		matrix_button_impl.low_level_deinit();

	memset(&matrix_button_impl, 0, sizeof(matrix_button_impl));

	return 0;
}

static void matrix_button_start(matrix_button_handle *handle)
{
	matrix_button_obj *obj = GET_BASE(handle);

	obj->en = 1;
}

static void matrix_button_stop(matrix_button_handle *handle)
{
	matrix_button_obj *obj = GET_BASE(handle);

	obj->en = 0;
}

static int matrix_button_get_state(matrix_button_handle *handle)
{
	matrix_button_obj *obj = GET_BASE(handle);

	return obj->state == PRESS ? PRESS : RELEASE;
}

static int matrix_button_destroy(matrix_button_handle * handle)
{
	matrix_button_obj *obj = GET_BASE(handle);
	matrix_button_obj *obj_t;

	if (obj == current_tiggered_obj) {
		MATRIX_BUTTON_ERR("the matrix button is working, can not destroy");
		return -1;
	}

	if (list_empty(&matrix_buttons_head))
		return -1;

	/* delete and free the matrix button object from matrix_buttons list head */
	list_for_each_entry(obj_t, &matrix_buttons_head, node) {
		if (obj_t == obj) {
			list_del(&obj->node);
			free(obj);
			return 0;
		}
	}

	return 0;
}

/**
  * @brief Create one short matrix button.
  * @note The short matrix button has only the RELEASE state. When it pressed, the
  *       callback function of the matrix button object will not be called.
  *       When it released, the callback function will be called and pass the
  *       RELEASE state immediately.
  * @param id_mask: the matrix button id
  * @retval The matrix button object handle, NULL if create failed.
  */
matrix_button_handle* create_short_matrix_button(uint32_t id_mask)
{
	uint32_t num = id_mask;
	int count = 0;

	/* record how many '1' in id_mask */
	while (num) {
		if (num & 1)
			count++;
		num >>= 1;
	}
	/* not support multiple matrix_buttons */
	if (count != 2) {
		MATRIX_BUTTON_ERR("short matrix button must only 1 matrix_buttons");
		return NULL;
	}

	matrix_button_obj *obj;

	/* check if the matrix button object exists */
	obj = get_matrix_buttons_obj(id_mask);
	if (obj) {
		MATRIX_BUTTON_ERR("matrix button %x object has already exist", id_mask);
		return NULL;
	}

	obj = (matrix_button_obj *) malloc(sizeof(matrix_button_obj));
	if (obj == NULL) {
		MATRIX_BUTTON_ERR("matrix button object malloc error");
		return NULL;
	}

	memset(obj, 0, sizeof(matrix_button_obj));
	obj->state = INVALID_STA;
	obj->id_mask = id_mask;
	obj->func = SHORT_BUTTON;
	obj->handle.start = matrix_button_start;
	obj->handle.stop = matrix_button_stop;
	obj->handle.get_state = matrix_button_get_state;
	obj->handle.destroy = matrix_button_destroy;

	/* add matrix button object to matrix_buttons list head */
	list_add_tail(&obj->node, &matrix_buttons_head);

	return &obj->handle;
}

/**
  * @brief Create one long matrix button.
  * @note The long matrix button has the PRESS/RELEASE state. When it pressed for
  *       more than timeout_ms milliseconds, the callback function of the matrix button
  *       object will be called and pass the PRESS state. When it released, the
  *       callback function will be called and pass the RELEASE state. If release
  *       the matrix button earlier than timeout_ms, nothing will happen, the callback
  *       function will not be called.
  * @param id_mask: the matrix button id
  *        timeout_ms：the timeout to trigger PRESS state.
  * @retval The matrix button object handle, NULL if create failed.
  */
matrix_button_handle* create_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms)
{
	uint32_t num = id_mask;
	int count = 0;

	while (num) {
		if (num & 1)
			count++;
		num >>= 1;
	}

	if (count != 2) {
		MATRIX_BUTTON_ERR("long matrix button must only 1 matrix_buttons");
		return NULL;
	}

	matrix_button_obj *obj;

	obj = get_matrix_buttons_obj(id_mask);
	if (obj) {
		MATRIX_BUTTON_ERR("matrix button %x object has already exist", id_mask);
		return NULL;
	}

	obj = (matrix_button_obj *) malloc(sizeof(matrix_button_obj));
	if (obj == NULL) {
		MATRIX_BUTTON_ERR("matrix button object malloc error");
		return NULL;
	}

	memset(obj, 0, sizeof(matrix_button_obj));
	obj->state = INVALID_STA;
	obj->id_mask = id_mask;
	obj->func = LONG_BUTTON;
	obj->timeout_ms = timeout_ms;
	obj->handle.start = matrix_button_start;
	obj->handle.stop = matrix_button_stop;
	obj->handle.get_state = matrix_button_get_state;
	obj->handle.destroy = matrix_button_destroy;

	list_add_tail(&obj->node, &matrix_buttons_head);

	return &obj->handle;
}

/**
  * @brief Create one short_long matrix button.
  * @note The short_long matrix button has the PRESS/RELEASE/REPEAT_RELEASE state.
  *       PRESS: if the matrix button pressed for more than timeout_ms(press_time >= timeout_ms),
  *              this state will be passed to callback function.
  *       RELEASE: if the matrix button released earlier than timeout_ms(press_time < timeout_ms),
  *                this state will be passed to callback function.
  *       REPEAT_RELEASE: if the matrix button pressed more than timeout_ms and released(press_time >= timeout_ms),
  *                       this state will be passed to callback function.
  * @param id_mask: the matrix button id
  *        timeout_ms：the timeout to trigger PRESS state.
  * @retval The matrix button object handle, NULL if create failed.
  */
matrix_button_handle* create_short_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms)
{
	uint32_t num = id_mask;
	int count = 0;

	while (num) {
		if (num & 1)
			count++;
		num >>= 1;
	}

	if (count != 2) {
		MATRIX_BUTTON_ERR("short and long matrix button must only 1 matrix_buttons");
		return NULL;
	}

	matrix_button_obj *obj;

	obj = get_matrix_buttons_obj(id_mask);
	if (obj) {
		MATRIX_BUTTON_ERR("matrix button %x object has already exist", id_mask);
		return NULL;
	}

	obj = (matrix_button_obj *) malloc(sizeof(matrix_button_obj));
	if (obj == NULL) {
		MATRIX_BUTTON_ERR("matrix button object malloc error");
		return NULL;
	}

	memset(obj, 0, sizeof(matrix_button_obj));
	obj->state = INVALID_STA;
	obj->id_mask = id_mask;
	obj->func = SHORT_LONG_BUTTON;
	obj->timeout_ms = timeout_ms;
	obj->handle.start = matrix_button_start;
	obj->handle.stop = matrix_button_stop;
	obj->handle.get_state = matrix_button_get_state;
	obj->handle.destroy = matrix_button_destroy;

	list_add_tail(&obj->node, &matrix_buttons_head);

	return &obj->handle;

}

/**
  * @brief Create one combined long matrix button.
  * @note The combined matrix button has the PRESS/RELEASE state. Combined long
  *       matrix button support multiple matrix_buttons, for example, key1|key2|key3, when all
  *       the three matrix_buttons are pressed and more than timeout_ms(all_press_time >= timeout_ms),
  *       the PRESS state will be passed to callback function. If any matrix button released,
  *       the RELEASE state will be passed to callback function. If any matrix button released
  *       earlier than timeout_ms(all_press_time < timeout_ms), nothing will happen.
  * @param id_mask: the matrix button id, support multiple matrix_buttons(KEY1|KEY2|KEY3).
  *        timeout_ms：the timeout to trigger PRESS state.
  * @retval The matrix button object handle, NULL if create failed.
  */
matrix_button_handle* create_combined_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms)
{
	uint32_t num = id_mask;
	int count = 0;

	while (num) {
		if (num & 1)
			count++;
		num >>= 1;
	}

	if (count < 3) {
		MATRIX_BUTTON_ERR("combined matrix button must have more than 2 matrix_buttons");
		return NULL;
	}

	matrix_button_obj *obj;

	obj = get_matrix_buttons_obj(id_mask);
	if (obj) {
		MATRIX_BUTTON_ERR("matrix button %x object has already exist", id_mask);
		return NULL;
	}

	obj = (matrix_button_obj *) malloc(sizeof(matrix_button_obj));
	if (obj == NULL) {
		MATRIX_BUTTON_ERR("matrix button object malloc error");
		return NULL;
	}

	memset(obj, 0, sizeof(matrix_button_obj));
	obj->state = INVALID_STA;
	obj->id_mask = id_mask;
	obj->func = COMBINED_LONG_BUTTON;
	obj->timeout_ms = timeout_ms;
	obj->handle.start = matrix_button_start;
	obj->handle.stop = matrix_button_stop;
	obj->handle.get_state = matrix_button_get_state;
	obj->handle.destroy = matrix_button_destroy;

	list_add_tail(&obj->node, &matrix_buttons_head);

	return &obj->handle;
}

/**
  * @brief Create one repeat long matrix button.
  * @note The repeat long matrix button has the PRESS/RELEASE/REPEAT_PRESS state.
  *       PRESS: if the matrix button pressed for more than timeout_ms(press_time >= timeout_ms),
  *              this state will be passed to callback function.
  *       RELEASE: if the matrix button pressed for more than timeout_ms and released,
  *                this state will be passed to callback function.
  *       REPEAT_PRESS: if the matrix button pressed all the time, the callback function
  *                     will be called in every repeat_timeout_ms, and REPEAT_PRESS
  *                     will be passed to callback.
  * @param id_mask: the matrix button id.
  *        timeout_ms：the timeout to trigger PRESS state.
  *        repeat_timeout_ms: the timeout to trigger REPEAT_PRESS state.
  * @retval The matrix button object handle, NULL if create failed.
  */
matrix_button_handle* create_repeat_long_matrix_button(uint32_t id_mask, uint32_t timeout_ms, uint32_t repeat_timeout_ms)
{
	uint32_t num = id_mask;
	int count = 0;

	while (num) {
		if (num & 1)
			count++;
		num >>= 1;
	}

	if (count != 2) {
		MATRIX_BUTTON_ERR("repeat long matrix button must only 1 matrix_buttons");
		return NULL;
	}

	matrix_button_obj *obj;

	obj = get_matrix_buttons_obj(id_mask);
	if (obj) {
		MATRIX_BUTTON_ERR("matrix button %x object has already exist", id_mask);
		return NULL;
	}

	obj = (matrix_button_obj *) malloc(sizeof(matrix_button_obj));
	if (obj == NULL) {
		MATRIX_BUTTON_ERR("matrix button object malloc error");
		return NULL;
	}

	memset(obj, 0, sizeof(matrix_button_obj));
	obj->state = INVALID_STA;
	obj->id_mask = id_mask;
	obj->func = REPEAT_LONG_BUTTON;
	obj->timeout_ms = timeout_ms;
	obj->repeat_timeout_ms = repeat_timeout_ms;
	obj->handle.start = matrix_button_start;
	obj->handle.stop = matrix_button_stop;
	obj->handle.get_state = matrix_button_get_state;
	obj->handle.destroy = matrix_button_destroy;

	list_add_tail(&obj->node, &matrix_buttons_head);

	return &obj->handle;
}

