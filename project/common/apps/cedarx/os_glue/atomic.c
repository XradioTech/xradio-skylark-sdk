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

#include <atomic.h>

signed long sync_fetch_and_add(volatile signed long *ptr, signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr += value;
    EXIT_CRITICAL();
    return old;
}

signed long sync_fetch_and_sub(volatile signed long *ptr, signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr -= value;
    EXIT_CRITICAL();

    return old;
}

signed long sync_fetch_and_or(volatile signed long *ptr,  signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr |= value;
    EXIT_CRITICAL();

    return old;
}

signed long sync_fetch_and_and(volatile signed long *ptr, signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr &= value;
    EXIT_CRITICAL();

    return old;
}

signed long sync_fetch_and_xor(volatile signed long *ptr, signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr ^= value;
    EXIT_CRITICAL();

    return old;
}

signed long sync_fetch_and_nand(volatile signed long *ptr, signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr &= value;
    *ptr = ~*ptr;
    EXIT_CRITICAL();

    return old;
}

signed long sync_add_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr += value;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_sub_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr -= value;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_or_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr |= value;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_and_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr &= value;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_xor_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr ^= value;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_nand_and_fetch(volatile signed long *ptr, signed long value)
{
    ENTER_CRITICAL();
    *ptr &= value;
    *ptr = ~*ptr;
    EXIT_CRITICAL();

    return *ptr;
}

signed long sync_lock_test_and_set(volatile signed long *ptr,signed long value)
{
    signed long old;

    ENTER_CRITICAL();
    old = *ptr;
    *ptr = value;
    EXIT_CRITICAL();

    return old;
}

int sync_bool_compare_and_swap(volatile signed long *ptr,signed long oldvalue, signed long newvalue)
{
    signed long old;
    int ret;
    //eLIBs_printf("__sync_bool_compare_and_swap!!!\n");
    ENTER_CRITICAL();
    old = *ptr;
    if(old == oldvalue)
    {
        ret = 1;
        *ptr = newvalue;
    }
    else
        ret = 0;
    EXIT_CRITICAL();

    return ret;
}
#endif
