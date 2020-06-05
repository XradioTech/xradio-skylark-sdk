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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "kernel/os/os.h"
#include "include/pthread.h"
#include "sys/list.h"
#include "driver/chip/hal_rtc.h"
#include "atomic.h"

#include "cdx_log.h"

#define CEDARX_HIGH_PRIO_EN        1

#define LOCK_CHECK_DEAD 0

#define DBG(fmt, ...)   //do{ printf("[Cedarx OS porting DBG] %s line %d, "fmt, __func__, __LINE__, ##__VA_ARGS__); } while(0)
#define INFO(fmt, ...)  do{ printf("[Cedarx OS porting INFO] %s line %d, "fmt, __func__, __LINE__, ##__VA_ARGS__); } while(0)
#define BUG()           do{ DBG("BUG happend!\n"); }while(0)

#define ALERT(fmt, ...) printf("[Cedarx OS porting ALERT] <%s> " fmt, __func__, ##__VA_ARGS__)

#define ENTRY()
#define INVALID() printf("[Cedarx OS porting INVALID] <%s> ", __func__)

typedef void *(*pthread_start_routine_pfn)(void*);

#define PTHREAD_STACK_DEFAULT_SIZE          (2 * 1024)

#define PTHREAD_DETACH_STATE        (1 << 0)
typedef struct {
    OS_Thread_t                 thd;
    pthread_osstate_t           state;
    struct list_head            task_list;
    OS_ThreadEntry_t            func;
    void *                      arg;
}xr_pthread_t;

static LIST_HEAD_DEF(g_pthread_task_head);
static OS_Mutex_t g_pthread_task_lock = {OS_INVALID_HANDLE};


pid_t gettid(void)
{
    ENTRY();
    return (pid_t)OS_ThreadGetCurrentHandle();
}

pthread_t pthread_self(void)
{
    ENTRY();

    struct list_head *pos;
    struct list_head *n;
    xr_pthread_t* iter = NULL;
    OS_ThreadHandle_t hdl = OS_ThreadGetCurrentHandle();

    OS_MutexLock(&g_pthread_task_lock, OS_WAIT_FOREVER);
    list_for_each_safe(pos, n, &g_pthread_task_head)
    {
        iter = list_entry(pos, xr_pthread_t, task_list);

        if(iter->thd.handle == hdl) {
            DBG("Find THREAD: %p\n", iter->thd.handle);
            OS_MutexUnlock(&g_pthread_task_lock);
            return (pthread_t)iter;
        }
    }
    OS_MutexUnlock(&g_pthread_task_lock);

    ALERT("NO WAY HERE!!!");

    return (pthread_t)NULL;
}

int pthread_mutex_init(pthread_mutex_t* mutex, const pthread_mutexattr_t* mutexattr)
{
    OS_Status ret;

    ENTRY();
    if (mutex == NULL)
        return EINVAL;
    if (mutexattr != NULL)
        ALERT("mutex set attribute\n");

    memset((void*)mutex, 0x00, sizeof(pthread_mutex_t));

    if ((ret = OS_RecursiveMutexCreate(&mutex->lock)) != OS_OK)
        return EINVAL;

    return 0;
}

int pthread_mutex_lock(pthread_mutex_t* mutex)
{
    ENTRY();
    OS_Status ret;

    if(mutex == NULL)
        return EINVAL;

    if ((ret = OS_RecursiveMutexLock(&mutex->lock, OS_WAIT_FOREVER)) != OS_OK) {
        return EINVAL;
    }

    return 0;
}

int pthread_equal(pthread_t a, pthread_t b)
{
    ENTRY();
    return ((xr_pthread_t*)a)->thd.handle == ((xr_pthread_t*)b)->thd.handle;
}


int pthread_mutex_unlock(pthread_mutex_t* mutex)
{
    ENTRY();

    if(mutex == NULL)
        return EINVAL;

    OS_RecursiveMutexUnlock(&mutex->lock);
    return 0;
}

int pthread_mutex_destroy(pthread_mutex_t* mutex)
{
    ENTRY();

    OS_Status ret;

    if(mutex == NULL)
        return EINVAL;

    if ((ret = OS_RecursiveMutexDelete(&mutex->lock)) != OS_OK)
        return EINVAL;
    memset((void*)mutex, 0x00, sizeof(pthread_mutex_t));

    return 0;
}

int pthread_mutex_timedlock(pthread_mutex_t* mutex, const struct timespec* tv)
{
    ENTRY();
    OS_Time_t ms;
    OS_Status ret;

    if(mutex == NULL)
        return EINVAL;

    ms = tv->tv_sec * 1000 + tv->tv_nsec / 1000000; //if sec = 0, nsec = 1?
    ms = (ms ? ms : 1);

    ret = OS_MutexLock(&mutex->lock, ms);
    if (ret == OS_E_TIMEOUT) {
        return ETIMEDOUT;
    } else if (ret != OS_OK) {
        return EINVAL;
    }

    return 0;
}

int pthread_mutex_trylock(pthread_mutex_t* mutex)
{
    ENTRY();
    if(mutex == NULL)
        return EINVAL;

    if (OS_RecursiveMutexLock(&mutex->lock, 0) != OS_OK)
        return EINVAL;

    return 0;
}


int pthread_mutexattr_destroy(pthread_mutexattr_t* mutexattr)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_mutexattr_getpshared(const pthread_mutexattr_t* mutexattr, int* shared)
{
    ENTRY();
    INVALID();
    return 0;
}
int pthread_mutexattr_gettype(const pthread_mutexattr_t* mutexattr, int* type)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_mutexattr_init(pthread_mutexattr_t* mutexattr)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_mutexattr_setpshared(pthread_mutexattr_t* mutexattr, int shared)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_mutexattr_settype(pthread_mutexattr_t* mutexattr, int type)
{
    ENTRY();
    INVALID();
    return 0;
}

int sem_init(sem_t *sem, int pshared, unsigned int value)
{
    ENTRY();
    OS_Status ret;
    unsigned short  user_cnt = (value & OS_SEMAPHORE_MAX_COUNT);

    if(sem == NULL)
        return -1;
    if (pshared != 0)
        ALERT("sem pshared!\n");
    if(value > OS_SEMAPHORE_MAX_COUNT)
        return -1;

    memset(sem, 0, sizeof(*sem));
    if ((ret = OS_SemaphoreCreate(&sem->sem, user_cnt, OS_SEMAPHORE_MAX_COUNT)) != OS_OK)
        return -1;

    sem->cnt = user_cnt;
    sem->wait = 0;

    return 0;
}

int sem_destroy(sem_t *sem)
{
    ENTRY();
    OS_Status ret;

    if(sem == NULL)
        return -1;

    ret = OS_SemaphoreDelete(&sem->sem);
    if(ret != OS_OK)
        return -1;

    return 0;
}

int sem_wait (sem_t *sem)
{
    ENTRY();
    OS_Status ret;

    if(sem == NULL)
        return -1;

    sem->wait++;
    ret = OS_SemaphoreWait(&sem->sem, OS_WAIT_FOREVER);
    sem->wait--;

    if(ret != OS_OK)
        return -1;
    sync_sub_and_fetch((volatile long *)&sem->cnt, 1);

    return 0;
}

int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout)
{
    ENTRY();
    OS_Status ret;
    int ms;
    struct timespec now;

    if(sem == NULL)
        return -1;

    clock_gettime(CLOCK_REALTIME, &now);
    ms = (abs_timeout->tv_sec - now.tv_sec) * 1000 + (abs_timeout->tv_nsec - now.tv_nsec) / 1000000;
    ms = (ms > 0) ? ms : 1;

    sem->wait++;
    ret = OS_SemaphoreWait(&sem->sem, ms);
    sem->wait--;
    if(ret == OS_E_TIMEOUT)
        return -ETIMEOUT;
    else if (ret != OS_OK)
        return -1;
    sync_sub_and_fetch((volatile long *)&sem->cnt, 1);

    return 0;
}

int sem_post(sem_t *sem)
{
    ENTRY();
    if(sem == NULL)
        return -1;

    sync_add_and_fetch((volatile long *)&sem->cnt, 1);
    OS_SemaphoreRelease(&sem->sem);

    return 0;
}

int sem_trywait(sem_t *sem)
{
    ENTRY();
    OS_Status ret;

    if(sem == NULL)
        return -1;

    sem->wait++;
    ret = OS_SemaphoreWait(&sem->sem, 0);
    sem->wait--;
    if(ret != OS_OK)
        return -1;
    sync_sub_and_fetch((volatile long *)&sem->cnt, 1);

    return 0;
}

int sem_getvalue(sem_t *sem, int *sval)
{
    ENTRY();

    if(sem == NULL || sval == NULL)
        return -1;

    *sval   = sem->cnt;//0 - ((int)sem->cnt);
    //DBG("sem value = %d\n", sem->cnt);

    return 0;
}

int sem_unlink(const char* name)
{
    ENTRY();
    INVALID();
    return -1;
}

int sem_close(sem_t *sem)
{
    ENTRY();
    INVALID();
    return -1;
}

sem_t *sem_open(const char *name, int oflag, ...)
{
    ENTRY();
    INVALID();
    return NULL;
}

int SemTimedWait(sem_t* sem, int64_t time_ms)
{
    OS_Status ret;

    if(sem == NULL)
          return -1;

    sem->wait++;
    ret = OS_SemaphoreWait(&sem->sem, time_ms);
    sem->wait--;

    if(ret == OS_E_TIMEOUT)
        return -ETIMEOUT;
    else if (ret != OS_OK)
        return -1;

    sync_sub_and_fetch((volatile long *)&sem->cnt, 1);

    return 0;
}

#if 1
static void thread_wrapper(void *p_arg)
{
    ENTRY();
    xr_pthread_t *wrap = p_arg;

    wrap->func(wrap->arg);

    DBG("thread exit\n");
    pthread_exit(NULL);
    return;
}


int pthread_once(pthread_once_t* once_obj, void (*init_func)(void))
{
    ENTRY();
    volatile pthread_once_t* once_obj_ptr = once_obj;

    if((once_obj == NULL) || (init_func == NULL))
    {
        // all pthread_once return zero.
        return 0;
    }

    // lock
    if (*once_obj_ptr != 0) {
        while ((*once_obj_ptr & ONCE_COMPLETED) == 0)
            OS_MSleep(10);
        return 0;
    }
    *once_obj_ptr = ONCE_INITIALIZING;
    //unlock


/*  if(((*once_obj_ptr & ONCE_COMPLETED) != 0))
    {
        return 0;
    }

    while(1)
    {
        signed long  old_value, new_value;

        do
        {
            old_value = *once_obj_ptr;
            if ((old_value & ONCE_COMPLETED) != 0)
            {
                break;
            }

            new_value = old_value | ONCE_INITIALIZING;
        } while (__sync_bool_compare_and_swap((volatile signed long *)once_obj_ptr, old_value, new_value) != 0);


        if ((old_value & ONCE_COMPLETED) != 0)
        {
            return 0;
        }

        if ((old_value & ONCE_INITIALIZING) == 0)
        {
            break;
        }

        //todo, goes into wait state.
        while(!(*once_obj_ptr & ONCE_COMPLETED))
        {
            OS_MSleep(1);
        }
    }*/

    (*init_func)();
    *once_obj_ptr = ONCE_COMPLETED;

    return 0;
}
int pthread_create(pthread_t* tid, pthread_attr_t const* attr, void *(*start_routine)(void*), void* arg)
{
    ENTRY();
    OS_Priority prio;
    OS_Status ret;
    xr_pthread_t *thrd;

#if CEDARX_HIGH_PRIO_EN
    prio = OS_PRIORITY_ABOVE_NORMAL;
#else
    prio =  OS_THREAD_PRIO_APP;
#endif

    if(tid == NULL)
        return -1;
    if(start_routine == NULL)
        return -1;
/*  if(attr != NULL)
        INVALID("don't support create a thread with attr\n");*/

    thrd = malloc(sizeof(xr_pthread_t));
    if(thrd == NULL)
        return -1;
    memset((void*)thrd, 0x00, sizeof(xr_pthread_t));

    INIT_LIST_HEAD(&thrd->task_list);
    *tid = (pthread_t)thrd;
    thrd->state = PTHREAD_TASK_STATE_READY;
    thrd->func = (OS_ThreadEntry_t)start_routine;
    thrd->arg = arg;

#ifdef NO_WRAP_THREAD
    ret = OS_ThreadCreate(&thrd)->thd,
                          "pthread",
                          (OS_ThreadEntry_t)start_routine,
                          arg,
                          prio,
                          (attr == NULL) ? PTHREAD_STACK_DEFAULT_SIZE : attr->stack_size);
#else
    ret = OS_ThreadCreate(&thrd->thd,
                          "pthread",
                          (OS_ThreadEntry_t)thread_wrapper,
                          thrd,
                          prio,
                          (attr == NULL) ? PTHREAD_STACK_DEFAULT_SIZE : attr->stack_size);
#endif

    if(ret != OS_OK)
    {
        ALERT("thread create failed!\n");
        free(thrd);
        return -1;
    } else
        INFO("thread: 0x%x create success!\n", (int)thrd->thd.handle);

    if (!OS_MutexIsValid(&g_pthread_task_lock))
        OS_MutexCreate(&g_pthread_task_lock);

    OS_MutexLock(&g_pthread_task_lock, OS_WAIT_FOREVER);
    list_add(&thrd->task_list, &g_pthread_task_head);
    OS_MutexUnlock(&g_pthread_task_lock);

    return 0;
}

int pthread_join(pthread_t tid, void** arg)
{
    ENTRY();
    struct list_head *pos;
    struct list_head *n;
    xr_pthread_t* iter = NULL;
    int exit = 0;
    int timeout = 20000;

    while (!exit) {
        exit = 1;
        OS_MutexLock(&g_pthread_task_lock, OS_WAIT_FOREVER);
        list_for_each_safe(pos, n, &g_pthread_task_head)
        {
            iter = list_entry(pos, xr_pthread_t, task_list);
            if(iter == (xr_pthread_t*)tid) {
                exit = 0;
                break;
/*                while(OS_ThreadIsValid(p))
                {
                    OS_MSleep(10);
                }
                return 0;
*/
            }
        }
        OS_MutexUnlock(&g_pthread_task_lock);

        OS_MSleep(10);
        timeout -= 10;
        if (timeout <= 0) {
            ALERT("thread %p can not exit\n", ((xr_pthread_t*)tid)->thd.handle);
            return -1;
        }
    }


/*  if(!OS_ThreadIsValid(tid))
    {
        //already detached.
        return -1;
    }

    while(OS_ThreadIsValid(tid))
    {
        OS_MSleep(10);
    }*/

/*  if(arg)
        *arg = iter->retval;*/

    return 0;
}

void pthread_exit(void* arg)
{
    ENTRY();
    if (arg != NULL)
        ALERT("pthread_exit pass arg to pthread join is not realized\n");

    struct list_head *pos;
    struct list_head *n;
    xr_pthread_t* iter = NULL;
    OS_ThreadHandle_t hdl = OS_ThreadGetCurrentHandle();

    OS_MutexLock(&g_pthread_task_lock, OS_WAIT_FOREVER);
    list_for_each_safe(pos, n, &g_pthread_task_head)
    {
        iter = list_entry(pos, xr_pthread_t, task_list);

        if(iter->thd.handle == hdl) {
            DBG("THREAD EXIT: %p\n", iter->thd.handle);
            list_del(pos);
            OS_Thread_t temp_thd = iter->thd;
            free(iter);
            OS_MutexUnlock(&g_pthread_task_lock);
            OS_ThreadDelete(&temp_thd);
            break;
        }
    }
    OS_MutexUnlock(&g_pthread_task_lock);

    return;
}


//in detach state on rtos default.
int pthread_detach(pthread_t tid)
{
    ENTRY();
    return 0;
}
#endif

int pthread_condattr_destroy(pthread_condattr_t* attr)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_condattr_getpshared(const pthread_condattr_t* attr, int* shared)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_condattr_init(pthread_condattr_t* attr)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_condattr_setpshared(pthread_condattr_t* attr, int shared)
{
    ENTRY();
    INVALID();
    return 0;
}

int pthread_cond_broadcast(pthread_cond_t* a)
{
    ENTRY();
    int wait;

    if(a == NULL)
        return -1;

    wait = a->wait;
    if (wait == 0)
        return 0;
    DBG("cond broadcast cnt = %d\n", wait);
    while (wait--)
        sem_post(a);


    return 0;
}

int pthread_cond_destroy(pthread_cond_t *a)
{
    ENTRY();
    if (a == NULL)
        return -1;

    sem_destroy(a);

    return 0;
}

int pthread_cond_init(pthread_cond_t* a, const pthread_condattr_t* b)
{
    ENTRY();
    if (a == NULL)
        return -1;

    if (b != NULL)
        ALERT("cond attr don't support\n");

    if(sem_init(a, 0, 0) != 0)
    {
        return -1;
    }

    return 0;
}

int pthread_cond_signal(pthread_cond_t* a)
{
    ENTRY();
    int wait;

    if(a == NULL)
        return -1;

    wait = a->wait;
    if (wait == 0)
        return 0;

    if(sem_post(a) != 0)
        return -1;

    return 0;
}

int pthread_cond_timedwait(pthread_cond_t* a, pthread_mutex_t* b, const struct timespec* c)
{
    ENTRY();
    unsigned char err = 0;

    if(a == NULL || b == NULL)
        return -1;

/*  ticks = (c->tv_sec * 1000 + c->tv_nsec / 1000000) / 10;
    pthread_mutex_unlock(b);
    esKRNL_FlagPend(*a, PTHREAD_EVENT_SET_0, ((OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME)<<16) | ticks, &err);*/

    struct timespec ts;
    int64_t ms;

    ENTER_CRITICAL();
    clock_gettime(CLOCK_REALTIME, &ts);
    ms = (ts.tv_sec - c->tv_sec) * 1000 + (ts.tv_nsec - c->tv_nsec) / 1000000;
    EXIT_CRITICAL();

    DBG("cond:ms: %d\n", (int)ms);

    OS_Status ret;

    a->wait++;
    pthread_mutex_unlock(b);
    ret = OS_SemaphoreWait(&a->sem, ms);
    a->wait--;

    if(ret == OS_E_TIMEOUT)
        err = -ETIMEOUT;
    else if (ret != OS_OK)
        err = -1;

    sync_sub_and_fetch((volatile long *)&a->cnt, 1);

    pthread_mutex_lock(b);

    return err;
}

int pthread_cond_wait(pthread_cond_t* a, pthread_mutex_t* b)
{
    int ret = 0;
    ENTRY();
    if(a == NULL || b == NULL)
        return -1;

    a->wait++;
    pthread_mutex_unlock(b);
    ret = OS_SemaphoreWait(&a->sem, OS_WAIT_FOREVER);
    a->wait--;

    if(ret != OS_OK)
        ret = -1;
    sync_sub_and_fetch((volatile long *)&a->cnt, 1);

    pthread_mutex_lock(b);
    return ret;
}


unsigned int osal_get_cur_tick(void)
{
    ENTRY();
    INVALID();
    return 0;
}

void abort(void)
{
    while(1)
    {
        OS_MSleep(1000);
        BUG();
    }
}

int clock_gettime(int a,struct timespec *t)
{
    ENTRY();
    if (a == CLOCK_REALTIME) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        t->tv_sec = tv.tv_sec;
        t->tv_nsec = tv.tv_usec * 1000;
    }
    else if (a == CLOCK_MONOTONIC) {
        long ms;
        ms = OS_TicksToMSecs(OS_GetTicks());
        t->tv_sec = ms / 1000;
        t->tv_nsec = (ms % 1000) * 1000000;
    }
    // not a standard realize.
    return 0;
}


/*
void exit(int status)
{
    ENTRY();
    INVALID();
}
*/
#endif

