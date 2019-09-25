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
#include "kernel/os/os.h"
#include "driver/chip/hal_rtc.h"
#include <math.h>

#define OFFSET_YEAR										2000
#define GET_AD_YEAR(counterYear)						(counterYear + OFFSET_YEAR)
#define GET_COUNTER_YEAR(adYear)						(adYear - OFFSET_YEAR)
#define RTC_SET_LEAP_YEAR(isLeapYear)					HAL_RTC_SetLeapYear(isLeapYear)
#define RTC_SET_DDHHMMSS(wday, hour, minute, second)	HAL_RTC_SetDDHHMMSS(wday, hour, minute, second)
#define RTC_SET_YYMMDD(isLeapYear, year, month, mday)	HAL_RTC_SetYYMMDD(isLeapYear, year, month, mday)
#define RTC_STOP_WDAY_ALARM(void)						HAL_RTC_StopWDayAlarm(void)
#define RTC_STOP_SEC_ALARM(void)						HAL_RTC_StopSecAlarm(void)

void rtc_get_yymmdd(uint8_t *isLeapYear, uint8_t *year, uint8_t *month, uint8_t *mday)
{
	HAL_RTC_GetYYMMDD(isLeapYear, year, month, mday);
}

void rtc_get_ddhhmmss(RTC_WeekDay *wday, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
	HAL_RTC_GetDDHHMMSS(wday, hour, minute, second);
}

void analysis_wday(RTC_WeekDay wday, char *buf)
{
	if (wday == 0)
		sprintf(buf, "%s", "monday");
	else if (wday == 1)
		sprintf(buf, "%s", "tuesday");
	else if (wday == 2)
		sprintf(buf, "%s", "wednesday");
	else if (wday == 3)
		sprintf(buf, "%s", "thursday");
	else if (wday == 4)
		sprintf(buf, "%s", "friday");
	else if (wday == 5)
		sprintf(buf, "%s", "saturday");
	else if (wday == 6)
		sprintf(buf, "%s", "sunday");
}

/**
 * @brief Determine whether it is a leap year
 * @param[in] adYear : AD. year
 * @return if it is a leap year return 1 else 0
 */
uint8_t is_leap_year(uint16_t adYear)
{
	if ((adYear % 4 == 0 && adYear % 100 != 0)
		|| (adYear % 400 == 0 && adYear % 3200 != 0)
		|| (adYear % 172800 == 0))
		return 1;
	else
		return 0;
}

/**
 * @brief  transform real AD year to counter year and Set the RTC date,
 * including leaf year flag, year, month and month day
 * @param[in] adYear : real AD year
 * @return :  return value < 0 on erro
 *
 * @note adYear = counter year + OFFSET_YEAR, counter year  set to [0, 255]
 */
int rtc_set_yymmdd(uint16_t adYear, uint8_t month, uint8_t mday)
{
	if (GET_COUNTER_YEAR(adYear) < 0) {
		printf("\n\nERRO: adYear should be greater than OFFSET_YEAR!\n\n");
		return -1;
	} else if (GET_COUNTER_YEAR(adYear) > 255) {
		printf("\n\nERRO: adYear should be less than (OFFSET_YEAR+255)!\n\n");
		return -2;
	}

	RTC_SET_YYMMDD(is_leap_year(adYear), GET_COUNTER_YEAR(adYear), month, mday);

	return 0;
}

void rtc_set_time(void)
{
	/*set time : year mouth day hour minute second*/
	printf("set time : 2004-2-28, saturday, 23:59:50\n");
	rtc_set_yymmdd(2004, 2, 28);
	RTC_SET_DDHHMMSS(RTC_WDAY_SATURDAY, 23, 59, 50);
}

void rtc_read_time(void)
{
	uint8_t leap, counterYear, mouth, mday;
	uint16_t adYear;

	RTC_WeekDay wday;
	uint8_t hour, minute, second;
	char buf[10];

	printf("read time:\n");
	rtc_get_yymmdd(&leap, &counterYear, &mouth, &mday);
	rtc_get_ddhhmmss(&wday, &hour, &minute, &second);

	analysis_wday(wday, buf);
	adYear = GET_AD_YEAR(counterYear);

	if (is_leap_year(adYear))
		printf("Is Leap Year\n");
	printf("%d-%d-%d, ", adYear, mouth, mday);
	printf("%s,%d:%d:%d\n", buf, hour, minute, second);
}

void rtc_reset_leap_year(void)
{
	uint8_t isLeapYear, counterYear, mouth, day;
	uint16_t adYear;

	rtc_get_yymmdd(&isLeapYear, &counterYear, &mouth, &day);
	adYear = GET_AD_YEAR(counterYear);
	isLeapYear = is_leap_year(adYear);
	RTC_SET_LEAP_YEAR(isLeapYear);
}

void wday_alarm_callback(void *arg)
{
	rtc_read_time();
	RTC_STOP_WDAY_ALARM();
	printf("wday alarm is arrive!!!\n\n");
}

void wday_alarm(void)
{
	RTC_WDayAlarmStartParam wday_param;

	printf("\nset wday alarm next day 0:0:20 \n");
	wday_param.alarmHour = 0;
	wday_param.alarmMinute = 0;
	wday_param.alarmSecond = 20;
	wday_param.alarmWDayMask = RTC_WDAY_ALARM_EN_BIT(RTC_WDAY_SUNDAY);
	wday_param.arg = NULL;
	wday_param.callback = wday_alarm_callback;

	HAL_RTC_StartWDayAlarm(&wday_param);
}

void sec_alarm_callback(void *arg)
{
	rtc_read_time();
	printf("second alarm is arrive!!!\n\n");
}

void sec_alarm(void)
{
	uint32_t alarmSecond;
	RTC_SecAlarmStartParam secParam;

	alarmSecond = 20;
	printf("set sec alarm %us after\n", alarmSecond);
	secParam.alarmSeconds = alarmSecond;
	secParam.arg = NULL;
	secParam.callback = sec_alarm_callback;
	HAL_RTC_StartSecAlarm(&secParam);
}

/**
 * @brief Read the time value (in microsecond) of the RTC's Free running counter
 *
 * Free running counter is a 48-bit counter which is driven by LFCLK and starts
 * to count as soon as the system reset is released and the LFCLK is ready.
 *
 * @note can't printf more than 32 bit digital
 */
void read_free_run_time(void)
{
	uint64_t freeRunTime = 0;

	freeRunTime = HAL_RTC_GetFreeRunTime();
	if (freeRunTime < pow(2, 32))
		printf("ReadFreeRunTime: %u uS\n", (uint32_t)freeRunTime);
}

void show_run_time(void)
{
	static int count = 40;
	uint8_t isShow = 1;

	printf("rtc show run time %d times start.\n", count);

	while (isShow) {
		if (count > 0) {
			read_free_run_time();
			count--;
		} else
			isShow = 0;
		OS_Sleep(1);
	}

	printf("rtc show run time over.\n");
}

int rtc_init(void)
{
	rtc_set_time();
	rtc_read_time();

	wday_alarm();
	sec_alarm();
	show_run_time();

	return 0;
}

void rtc_deinit(void)
{
	RTC_STOP_SEC_ALARM();
	RTC_STOP_WDAY_ALARM();
}

/* Run this example, please connect the uart0 */
int main(void)
{
	printf("rtc example started\n\n");
	/*waiting for  rtc stability*/
	OS_MSleep(200);

	rtc_init();

	while (1) {
		OS_Sleep(10);
		rtc_reset_leap_year();
	}
	rtc_deinit();

	return 0;
}
