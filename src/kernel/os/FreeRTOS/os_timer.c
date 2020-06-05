/**
 * @file os_timer.c
 * @author XRADIO IOT WLAN Team
 */

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

#include "kernel/os/os_timer.h"
#include "os_util.h"
#include "timers.h"

/* TODO: what block time should be used ? */
#define OS_TIMER_WAIT_FOREVER	portMAX_DELAY
#define OS_TIMER_WAIT_NONE		0

#if (defined(configUSE_TIMER_ID_AS_CALLBACK_ARG) && configUSE_TIMER_ID_AS_CALLBACK_ARG == 1)
#define OS_TIMER_USE_FREERTOS_ORIG_CALLBACK	0
#else
#define OS_TIMER_USE_FREERTOS_ORIG_CALLBACK	1
#endif

#if OS_TIMER_USE_FREERTOS_ORIG_CALLBACK

/* Timer private data definition */
typedef struct OS_TimerPriv {
    TimerHandle_t       handle;   /* Timer handle */
    OS_TimerCallback_t  callback; /* Timer expire callback function */
    void               *argument; /* Argument of timer expire callback function */
} OS_TimerPriv_t;

static void OS_TimerPrivCallback(TimerHandle_t xTimer)
{
	OS_TimerPriv_t *priv;

	priv = pvTimerGetTimerID(xTimer);
	if (priv && priv->callback) {
		priv->callback(priv->argument);
	} else {
		OS_WRN("Invalid timer callback\n");
	}
}

OS_Status OS_TimerCreate(OS_Timer_t *timer, OS_TimerType type,
                         OS_TimerCallback_t cb, void *arg, uint32_t periodMS)
{
	OS_TimerPriv_t *priv;

	OS_HANDLE_ASSERT(!OS_TimerIsValid(timer), timer->handle);

	priv = OS_Malloc(sizeof(OS_TimerPriv_t));
	if (priv == NULL) {
		return OS_E_NOMEM;
	}

	priv->callback = cb;
	priv->argument = arg;
	priv->handle = xTimerCreate("",
	                            OS_MSecsToTicks(periodMS),
	                            type == OS_TIMER_PERIODIC ? pdTRUE : pdFALSE,
	                            priv,
	                            OS_TimerPrivCallback);
	if (priv->handle == NULL) {
		OS_ERR("err %"OS_HANDLE_F"\n", priv->handle);
		OS_Free(priv);
		return OS_FAIL;
	}
	timer->handle = priv;
	return OS_OK;
}

static __inline TimerHandle_t OS_TimerGetKernelHandle(OS_Timer_t *timer)
{
	OS_TimerPriv_t *priv = timer->handle;
	return priv->handle;
}

#else /* OS_TIMER_USE_FREERTOS_ORIG_CALLBACK */

OS_Status OS_TimerCreate(OS_Timer_t *timer, OS_TimerType type,
                         OS_TimerCallback_t cb, void *arg, uint32_t periodMS)
{
	OS_HANDLE_ASSERT(!OS_TimerIsValid(timer), timer->handle);

	timer->handle = xTimerCreate("",
	                             OS_MSecsToTicks(periodMS),
	                             type == OS_TIMER_PERIODIC ? pdTRUE : pdFALSE,
	                             arg,
	                             cb);
	if (timer->handle == NULL) {
		OS_ERR("err %"OS_HANDLE_F"\n", timer->handle);
		return OS_FAIL;
	}
	return OS_OK;
}

static __inline TimerHandle_t OS_TimerGetKernelHandle(OS_Timer_t *timer)
{
	return timer->handle;
}

#endif /* OS_TIMER_USE_FREERTOS_ORIG_CALLBACK */

OS_Status OS_TimerDelete(OS_Timer_t *timer)
{
	TimerHandle_t handle;
	BaseType_t ret;

	OS_HANDLE_ASSERT(OS_TimerIsValid(timer), timer->handle);

	handle = OS_TimerGetKernelHandle(timer);
	ret = xTimerDelete(handle, OS_TIMER_WAIT_FOREVER);
	if (ret != pdPASS) {
		OS_ERR("err %"OS_BASETYPE_F"\n", ret);
		return OS_FAIL;
	}

#if OS_TIMER_USE_FREERTOS_ORIG_CALLBACK
	OS_TimerPriv_t *priv = timer->handle;
#endif
	OS_TimerSetInvalid(timer);
#if OS_TIMER_USE_FREERTOS_ORIG_CALLBACK
	OS_Free(priv);
#endif
	return OS_OK;
}

OS_Status OS_TimerStart(OS_Timer_t *timer)
{
	TimerHandle_t handle;
	BaseType_t ret;
	BaseType_t taskWoken;

	OS_HANDLE_ASSERT(OS_TimerIsValid(timer), timer->handle);

	handle = OS_TimerGetKernelHandle(timer);

	if (OS_IsISRContext()) {
		taskWoken = pdFALSE;
		ret = xTimerStartFromISR(handle, &taskWoken);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
		portEND_SWITCHING_ISR(taskWoken);
	} else {
		ret = xTimerStart(handle, OS_TIMER_WAIT_NONE);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
	}

	return OS_OK;
}

OS_Status OS_TimerChangePeriod(OS_Timer_t *timer, uint32_t periodMS)
{
	TimerHandle_t handle;
	BaseType_t ret;
	BaseType_t taskWoken;

	OS_HANDLE_ASSERT(OS_TimerIsValid(timer), timer->handle);

	handle = OS_TimerGetKernelHandle(timer);

	if (OS_IsISRContext()) {
		taskWoken = pdFALSE;
		ret = xTimerChangePeriodFromISR(handle, periodMS, &taskWoken);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
		portEND_SWITCHING_ISR(taskWoken);
	} else {
		ret = xTimerChangePeriod(handle, periodMS, OS_TIMER_WAIT_NONE);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
	}

	return OS_OK;
}

OS_Status OS_TimerStop(OS_Timer_t *timer)
{
	TimerHandle_t handle;
	BaseType_t ret;
	BaseType_t taskWoken;

	OS_HANDLE_ASSERT(OS_TimerIsValid(timer), timer->handle);

	handle = OS_TimerGetKernelHandle(timer);

	if (OS_IsISRContext()) {
		taskWoken = pdFALSE;
		ret = xTimerStopFromISR(handle, &taskWoken);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
		portEND_SWITCHING_ISR(taskWoken);
	} else {
		ret = xTimerStop(handle, OS_TIMER_WAIT_FOREVER);
		if (ret != pdPASS) {
			OS_ERR("err %"OS_BASETYPE_F"\n", ret);
			return OS_FAIL;
		}
	}

	return OS_OK;
}

int OS_TimerIsActive(OS_Timer_t *timer)
{
	TimerHandle_t handle;

	if (!OS_TimerIsValid(timer)) {
		return 0;
	}

	handle = OS_TimerGetKernelHandle(timer);

	return (xTimerIsTimerActive(handle) != pdFALSE);
}
