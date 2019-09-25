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
#include "pm/pm.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_wakeup.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"

#define TEST_SLEEP          1
#define TEST_STANDBY        1
#define TEST_HIBERNATION    1
#define TEST_STANDBY_DTIM   0

#define WAKEUP_IO_PIN_DEF   (5)
#define WAKEUP_IO_MODE_DEF  (WKUPIO_WK_MODE_FALLING_EDGE)
#define WAKEUP_IO_PULL_DEF  (GPIO_PULL_UP)
#define BUTTON_WAKEUP_PORT_DEF GPIO_PORT_A
#define BUTTON_WAKEUP_PIN_DEF  WakeIo_To_Gpio(WAKEUP_IO_PIN_DEF)

static uint16_t wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;
observer_base *g_ob;

/**
 * @brief wlan msg callback.
 *
 * @retval None.
 */
static void wlan_msg_recv(uint32_t event, uint32_t data, void *arg)
{
    uint16_t type = EVENT_SUBTYPE(event);
    printf("%s msg type:%d\n", __func__, type);

    switch (type) {
    case NET_CTRL_MSG_WLAN_CONNECTED:
        wlan_event = NET_CTRL_MSG_WLAN_CONNECTED;
        break;
    case NET_CTRL_MSG_WLAN_DISCONNECTED:
        wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;
        break;
    case NET_CTRL_MSG_WLAN_SCAN_SUCCESS:
    case NET_CTRL_MSG_WLAN_SCAN_FAILED:
    case NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED:
    case NET_CTRL_MSG_WLAN_CONNECT_FAILED:
        break;
    case NET_CTRL_MSG_CONNECTION_LOSS:
        wlan_event = WLAN_EVENT_CONNECTION_LOSS;
        break;
    case NET_CTRL_MSG_NETWORK_UP:
        wlan_event = NET_CTRL_MSG_NETWORK_UP;
        break;
    case NET_CTRL_MSG_NETWORK_DOWN:
        wlan_event = NET_CTRL_MSG_NETWORK_DOWN;
        break;
#if (!defined(__CONFIG_LWIP_V1) && LWIP_IPV6)
    case NET_CTRL_MSG_NETWORK_IPV6_STATE:
        break;
#endif
    default:
        printf("unknown msg (%u, %u)\n", type, data);
        break;
    }
}

/**
 * @brief init register wlan msg callback function.
 *
 * @param none
 *
 * @retval None.
 */
static int wlan_msg_init(void)
{
    g_ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
                                        NET_CTRL_MSG_ALL,
                                        wlan_msg_recv,
                                        NULL);
    if (g_ob == NULL)
        return -1;
    if (sys_ctrl_attach(g_ob) != 0)
        return -1;

    return 0;
}

/**
 * @brief deinit wlan mes callback function.
 *
 * @param none
 *
 * @retval None.
 */
static int wlan_msg_deinit(void)
{
    if (g_ob == NULL)
        return -1;
    if (sys_ctrl_detach(g_ob) != 0)
        return -1;

    return 0;
}

/**
 * @brief check wlan connect AP which will keep work at standby mode only.
 *
 * @param none
 *
 * @retval None.
 *
 * @other
 * cmds:
 * 1. net sta config ap_ssid [ap_psk]
 * 2. net sta enable
 * 3. netcmd lmac vif0_set_pm_dtim 8
 *    note: 8 should AP DTIM times(if AP DTIM is 3, set vif0_set_pm_dtim to 9)
 * 4. wifi will goto standby in 3S, and goto standby again if wakeup.
 */
static void pm_wakeup_wlan_connect_ap_attach()
{
#if PRJCONF_NET_EN
    wlan_msg_init();
    printf("\nPlease connect AP\n"
           "Example "
           "step1: net sta config TES_TPLINK_WDR6500#33 sw4wifionly  "
           "step2: net sta enable\n");
    while (!(g_wlan_netif && netif_is_up(g_wlan_netif) &&
             netif_is_link_up(g_wlan_netif))) {
        OS_MSleep(1000);
    }
    wlan_set_ps_mode(g_wlan_netif, 1);
    OS_MSleep(3000);
#else
    printf("\n\nPlease config open PRJCONF_NET_EN \n");
#endif
}

/**
 * @brief detach wlan message callback.
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_wlan_connect_ap_detach()
{
#if PRJCONF_NET_EN
    wlan_msg_deinit();
#else
    printf("\n\nPlease config open PRJCONF_NET_EN \n");
#endif
}

/**
 * @brief setup uart0 to wakeup source which will keep work at sleep mode only.
 *
 * @note uart irq only work in sleep mode
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_uartirq_init(void)
{
    HAL_UART_SetBypassPmMode(UART0_ID, PM_SUPPORT_SLEEP);
}

/**
 * @brief disable uart0 to wakeup source.
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_uartirq_deinit(void)
{
    HAL_UART_SetBypassPmMode(UART0_ID, 0);
}

/**
 * @brief setup wakeup source to timer which after 10 second will create irq.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_timer_init(void)
{
    /*wakeup timer 10 seconds*/
    HAL_Wakeup_SetTimer_mS(10000);
}

/**
 * @brief disable wakeup source to timer.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_timer_deinit(void)
{
    HAL_Wakeup_ClrTimer();
}

/**
 * @brief setup wakeup source to button which after 10 second will create irq.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_button_init(void)
{
    GPIO_InitParam param;
    param.driving = GPIO_DRIVING_LEVEL_1;
    param.pull = GPIO_PULL_UP;
    param.mode = GPIOx_Pn_F0_INPUT;
    HAL_GPIO_Init(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF, &param);

    /*Sakeup io debounce clock source 0  freq is LFCLK 32K*/
    HAL_PRCM_SetWakeupDebClk0(0);
    /*Wakeup IO 5 debounce clock select source 0*/
    HAL_PRCM_SetWakeupIOxDebSrc(WAKEUP_IO_PIN_DEF, 0);
    /*Wakeup IO 5 input debounce clock cycles is 16+1*/
    HAL_PRCM_SetWakeupIOxDebounce(WAKEUP_IO_PIN_DEF, 1);
    /*Wakeup IO 5 enable, negative edge,  */
    HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
}

/**
 * @brief disable wakeup source to button.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_button_deinit(void)
{
    /*Sakeup io debounce clock source 0  freq is LFCLK 32K to default*/
    HAL_PRCM_SetWakeupDebClk0(0);
    /*Wakeup IO 5 debounce clock select source 0 to default*/
    HAL_PRCM_SetWakeupIOxDebSrc(WAKEUP_IO_PIN_DEF, 0);
    /*Wakeup IO 5 input debounce clock cycles to default*/
    HAL_PRCM_SetWakeupIOxDebounce(WAKEUP_IO_PIN_DEF, 0);

    HAL_GPIO_DisableIRQ(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
    HAL_GPIO_DeInit(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
}


int main(void)
{
    platform_init();

    printf("\n\nPM example start!\n");
    printf("Support 3 low power modes: sleep/standby/hibernation\n");
    printf("Sleep support wakeup methods:       timer/button/devirq\n");
    printf("Standby support wakeup methods:     timer/button/wlan\n");
    printf("Hibernation support wakeup methods: timer/button\n\n\n");
    OS_MSleep(3000);

#if TEST_SLEEP
    /*enetr sleep test*/
    printf("\n\nEnter sleep mode, setup wakeup source uartirq&timer&button\n\n");
    pm_wakeup_uartirq_init();
    pm_wakeup_timer_init();
    pm_wakeup_button_init();
    pm_enter_mode(PM_MODE_SLEEP);
    printf("Wakeup event:%d\n", HAL_Wakeup_GetEvent());
    printf("Exit sleep mode\n\n");
    pm_wakeup_uartirq_deinit();
    pm_wakeup_timer_deinit();
    pm_wakeup_button_deinit();
#endif

#if TEST_STANDBY
    /*enetr standby test*/
    printf("\n\nEnter standby mode, setup wakeup source wlan&timer&button\n\n");
    pm_wakeup_wlan_connect_ap_attach();
    pm_wakeup_timer_init();
    pm_wakeup_button_init();
    pm_enter_mode(PM_MODE_STANDBY);
    printf("Wakeup event:%d\n", HAL_Wakeup_GetEvent());
    printf("Exit standby mode\n\n");
    pm_wakeup_timer_deinit();
    pm_wakeup_button_deinit();
    pm_wakeup_wlan_connect_ap_detach();
#endif

#if TEST_STANDBY_DTIM
    pm_wakeup_wlan_connect_ap_attach();
    pm_wakeup_button_init();
    wlan_sta_scan_interval(4);
    OS_MSleep(3000);
    while (1) {
        uint32_t wakeup_event = HAL_Wakeup_GetEvent();
        uint32_t end_time;

        printf("wakeup_event:%x\n", wakeup_event);
        if (wakeup_event & PM_WAKEUP_SRC_WLAN) {
            /* Wait wlan event or process recived data.
             * Need't wait 15ms after data has been processed, if
             *  you kwow this wakeup by recived data.
             * Or else wait 15mS to get detail wlan event( maybe
             *  CONNECTION_LOSS or DISCONNECTED).
             */
            OS_MSleep(15);
            end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + 180;
            /* maybe disconnect event, wait this event */
            while (((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
                    ((g_wlan_netif && netif_is_up(g_wlan_netif) &&
                      netif_is_link_up(g_wlan_netif))))&&
                   OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
                OS_MSleep(2);
            }
        }
        OS_MSleep(2);
        while ((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
               (!(g_wlan_netif && netif_is_up(g_wlan_netif) &&
                  netif_is_link_up(g_wlan_netif)))) {
            OS_MSleep(1);
        }
        pm_enter_mode(PM_MODE_STANDBY);
        printf("Exit standby mode\n\n");
        pm_wakeup_button_deinit();
        pm_wakeup_wlan_connect_ap_detach();
    }
#endif

#if TEST_HIBERNATION
    /*enetr hibernation test*/
    printf("\n\nEnter hibernantion mode, setup wakeup source button&timer\n\n");
    pm_wakeup_button_init();
    pm_wakeup_timer_init();
    pm_enter_mode(PM_MODE_HIBERNATION);
#endif

    return 0;
}
