#include "driver/drv_def.h"
#include "../hal_base.h"
#include "sys/io.h"
#include "pm/pm.h"
#include "drv_trng.h"

#if (__CONFIG_CHIP_ARCH_VER == 2)
#define TRNG_ALE	(DRV_ALE)
#define TRNG_ERR	(DRV_ERR)
#define TRNG_INF	(DRV_INF)
#define TRNG_DBG	(DRV_DBG)
#define TRNG_PRINT_LEVEL (TRNG_INF)
#define TRNG_PRINT(print_level, fmt, arg...) 	\
	do {										\
        if(print_level <= TRNG_PRINT_LEVEL)		\
            printf("[Trng] "fmt, ##arg);		\
    } while (0);

static void  __attribute__((unused)) trng_print_register(const int line, uint64_t* time)
{
	#if (TRNG_DBG <= TRNG_PRINT_LEVEL)
	printf("******************TRNG: %d**************************\n", line);
	for(int i=0; i<(0x50/4); i+=1) {
		if(i%4 == 0) printf("\n0x%8x:", (uint32_t)(&(TRNG->CTRL_CFG) + i));
		printf("  0x%8x", (uint32_t)(*(&(TRNG->CTRL_CFG) + i)));
	}
	printf("\n");
	(*time)+=(0x50/4)-1;
	#endif
}

static void trng_jitter_enable(uint32_t timing, uint32_t div, uint64_t* time)
{
	HAL_SET_BIT(TRNG->CTRL_CFG, CC_ENABLE_MASK | CC_RO_CRTL_MASK); /*trng_en = 1, trngro_ctrl = 0xff */
	HAL_MODIFY_REG(TRNG->JITTER_CNT_TIMING, JCT_JITTER_COUNTER_TIMING_MASK, (timing << JCT_JITTER_COUNTER_TIMING));
	HAL_SET_BIT(TRNG->JITTER_CFG, JC_JITTER_MONTITOR_WORK_EN_MASK | JC_JITTER_COUNTER_START_MASK);
	HAL_MODIFY_REG(TRNG->JITTER_CFG, JC_COUNTER_2DIV_MASK, (div << JC_COUNTER_2DIV));
	*time += 8;
}

static void trng_jitter_disable(uint64_t* time)
{
	HAL_CLR_BIT(TRNG->JITTER_CFG, JC_JITTER_MONTITOR_WORK_EN_MASK);
	TRNG->CTRL_CFG = 0;
	*time += 3;
}

static void trng_jitter_get_value(uint32_t* buf, uint64_t* time)
{
	(*time) += 2;
	while(!HAL_GET_BIT(TRNG->JITTER_COUNTER_READY, JCR_JITTER_COUNTER_READY_MASK)){
		HAL_UDelay(1);
		(*time) += 2;
	}
	buf[0]=TRNG->JITTER_CNT_RESULT0;
	buf[1]=TRNG->JITTER_CNT_RESULT1;
	buf[2]=TRNG->JITTER_CNT_RESULT2;
	buf[3]=TRNG->JITTER_CNT_RESULT3;
	buf[4]=TRNG->JITTER_CNT_RESULT4;
	buf[5]=TRNG->JITTER_CNT_RESULT5;
	buf[6]=TRNG->JITTER_CNT_RESULT6;
	buf[7]=TRNG->JITTER_CNT_RESULT7;
	*time += 8;
}

static void trng_monitor_config(uint32_t rtc, uint32_t apt_c, uint32_t apt_w, uint64_t* time)
{
	HAL_MODIFY_REG(TRNG->MONITOR_RCT, MR_RCT_C_MASK, (rtc << MR_RCT_C));
	HAL_MODIFY_REG(TRNG->MONITOR_APT, (MA_APT_C_MASK | MA_APT_W_MASK), (apt_c << MA_APT_C) | (apt_w << MA_APT_W));
	*time += 4;
}

static void trng_extract_config(uint32_t div1, uint32_t div2, uint32_t type, uint32_t ratio, uint64_t* time)
{
	HAL_MODIFY_REG(TRNG->EXTRACT_CFG, (EC_RO_SAMPLING_RATION0_MASK | EC_RO_SAMPLING_RATION1_MASK | EC_RESILIENT_TYPE_MASK | EC_RESILIENT_RATIO_MASK), \
					(div1 << EC_RO_SAMPLING_RATION0) | (div2 << EC_RO_SAMPLING_RATION1) | (type << EC_RESILIENT_TYPE) | (ratio << EC_RESILIENT_RATIO));
	(*time) += 2;
}

static void trng_enable(uint64_t* time)
{
	HAL_SET_BIT(TRNG->CTRL_CFG, (CC_ENABLE_MASK | CC_RO_CRTL_MASK));	//trng_en = 1, trngro_ctrl = 0xff
	HAL_SET_BIT(TRNG->MONITOR_RCT, MR_MONITOR_EN_MASK);					//trng_monitor_en = 1
	*time += 4;
}

static void trng_disable(uint64_t* time)
{
	HAL_CLR_BIT(TRNG->JITTER_CFG, JC_JITTER_MONTITOR_WORK_EN_MASK);
	TRNG->EXTRACT_CFG = 0;
	HAL_CLR_BIT(TRNG->MONITOR_RCT, MR_MONITOR_EN_MASK);
	TRNG->CTRL_CFG = 0;
	*time += 6;
}

static DRV_Status trng_extract_start(uint64_t* time)
{
	uint32_t state;
	HAL_SET_BIT(TRNG->EXTRACT_CFG, EC_EXTRACT_START_MASK);
	*time += 2;
	do {
		state = HAL_GET_BIT_VAL(TRNG->CTRL_CFG, CC_READY, CC_READY_VMASK);
		(*time)++;
		if(state > 1) {
			TRNG_PRINT(TRNG_ERR, "extract start error: state = %u\n", state);
			 return DRV_ERROR;
		}
	}while(state != 1);

	return DRV_OK;
}

static DRV_Status trng_get_value(uint32_t* buf, uint64_t* time)
{
	uint32_t state = HAL_GET_BIT_VAL(TRNG->CTRL_CFG, CC_READY, CC_READY_VMASK);
	DRV_Status ret = DRV_OK;
	(*time) ++;
	while(state){
		if(state > 1){
			ret = HAL_ERROR;
			break;
		}
		state = HAL_GET_BIT_VAL(TRNG->CTRL_CFG, CC_READY, CC_READY_VMASK);
		(*time) ++;
	}
	if(!state){
		buf[0]=TRNG->RAND_BIT_URN0;
		buf[1]=TRNG->RAND_BIT_URN1;
		buf[2]=TRNG->RAND_BIT_URN2;
		buf[3]=TRNG->RAND_BIT_URN3;
		*time += 4;
		ret = DRV_OK;
	}
	return ret;
}

void DRV_Trng_Init(uint64_t* time)
{
	uint32_t rtc = 1032;
	uint32_t apt_c = 1009;
	uint32_t apt_w = 1024;

	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_TRNG);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_TRNG);
    *time = DRV_Trng_Reg_Access_Time();
	trng_monitor_config(rtc, apt_c, apt_w, time);
}

void DRV_Trng_Deinit()
{
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_TRNG);
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_TRNG);
}

uint64_t DRV_Trng_Reg_Access_Time(void)
{
	return (((uint64_t)(TRNG->REG_ACCESS_CTR2) << 32) | (uint64_t)(TRNG->REG_ACCESS_CTR1));
}

DRV_Status DRV_Trng_Jitter_Test(uint64_t *actime)
{
	DRV_Status ret = DRV_OK;
	uint32_t timing = 5000;
	uint32_t trng_jitter_buff[8];

	TRNG_PRINT(TRNG_DBG, "trng_jitter_test start\n");
	trng_jitter_enable(timing, 0, actime);

	trng_jitter_get_value(trng_jitter_buff, actime);
	TRNG_PRINT(TRNG_DBG, "trng_jitter_buff[0] = %d\n", (int)trng_jitter_buff[0]);
	for(int i=1;i<8;i++) {
		TRNG_PRINT(TRNG_DBG, "trng_jitter_buff[%d] = %d\n", i, (int)trng_jitter_buff[i]);
		if(trng_jitter_buff[i] == trng_jitter_buff[i-1]) {
			ret = DRV_ERROR;
			goto out;
		}
	}

out:
	trng_jitter_disable(actime);
	return ret;
}

DRV_Status DRV_Trng_Extract(uint8_t type, uint32_t random[4], uint64_t *actime)
{
	DRV_Status ret = DRV_OK;
	uint32_t div1 = 0;
	uint32_t div2 = 0;
	uint32_t ratio = 127;

	if((NULL == random) || (type > 1)) {
		TRNG_PRINT(TRNG_ERR, "invalid argc\n");
		return DRV_INVALID;
	}

	trng_enable(actime);
	trng_extract_config(div1, div2, type, ratio, actime);
	ret = trng_extract_start(actime);
	if(ret != DRV_OK) {
		TRNG_PRINT(TRNG_ERR, "extract start error\n");
		ret = DRV_ERROR;
		goto out;
	}
	ret = trng_get_value(random, actime);
	if(ret != DRV_OK) {
		TRNG_PRINT(TRNG_ERR, "extract get_value error=\n");
		ret = DRV_ERROR;
		goto out;
	}
	TRNG_PRINT(TRNG_DBG, "trng result: %u %u %u %u\n", random[0], random[1], random[2], random[3]);
	trng_print_register(__LINE__, actime);
out:
	trng_disable(actime);
	return ret;
}

#endif /*(__CONFIG_CHIP_ARCH_VER == 2) */
