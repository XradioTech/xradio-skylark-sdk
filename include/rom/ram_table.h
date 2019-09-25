#ifndef _ROM_RAM_TABLE_H_
#define _ROM_RAM_TABLE_H_

#ifdef __CONFIG_ROM

#define RAM_TBL_IDX(name) RAM_TBL_IDX_##name

#define RAM_TBL_FUN(prototype, name) \
	((prototype)(ram_table[RAM_TBL_IDX(name)]))

#define RAM_TBL_DATA(type, name) \
	((type)(ram_table[RAM_TBL_IDX(name)]))

extern unsigned int ram_table[];

enum {
	/* internal used by rom only */

	/* chip */
	RAM_TBL_IDX(SYS_PLL_CLOCK),
	RAM_TBL_IDX(SYS_LFCLOCK),
	RAM_TBL_IDX(HAL_PRCM_GetHFClock),
	RAM_TBL_IDX(HAL_PRCM_GetLFClock),
	RAM_TBL_IDX(HAL_PRCM_GetDevClock),
	RAM_TBL_IDX(HAL_CCM_BusGetAPBSClock),

	/* lib */
	RAM_TBL_IDX(__wrap__malloc_r),
	RAM_TBL_IDX(__wrap__realloc_r),
	RAM_TBL_IDX(__wrap__free_r),
	RAM_TBL_IDX(_sbrk),
	RAM_TBL_IDX(__wrap_vprintf),
	RAM_TBL_IDX(__wrap_puts),
	RAM_TBL_IDX(__wrap_vfprintf),
	RAM_TBL_IDX(__wrap_fputs),
	RAM_TBL_IDX(__wrap_putchar),
	RAM_TBL_IDX(__wrap_putc),
	RAM_TBL_IDX(__wrap_fputc),
	RAM_TBL_IDX(__wrap_fflush),

	/* FreeRTOS */
	RAM_TBL_IDX(configMINIMAL_STACK_SIZE),
	RAM_TBL_IDX(configTIMER_QUEUE_LENGTH),
	RAM_TBL_IDX(configTIMER_TASK_STACK_DEPTH),
	RAM_TBL_IDX(vApplicationStackOverflowHook),

	/* OS */
	RAM_TBL_IDX(OS_SemaphoreCreate),
	RAM_TBL_IDX(OS_SemaphoreCreateBinary),
	RAM_TBL_IDX(OS_SemaphoreDelete),
	RAM_TBL_IDX(OS_SemaphoreWait),
	RAM_TBL_IDX(OS_SemaphoreRelease),
	RAM_TBL_IDX(OS_SemaphoreIsValid),
	RAM_TBL_IDX(OS_SemaphoreSetInvalid),

	RAM_TBL_IDX(OS_MutexCreate),
	RAM_TBL_IDX(OS_MutexDelete),
	RAM_TBL_IDX(OS_MutexLock),
	RAM_TBL_IDX(OS_MutexUnlock),
	RAM_TBL_IDX(OS_RecursiveMutexCreate),
	RAM_TBL_IDX(OS_RecursiveMutexDelete),
	RAM_TBL_IDX(OS_RecursiveMutexLock),
	RAM_TBL_IDX(OS_RecursiveMutexUnlock),

	RAM_TBL_IDX(OS_ThreadList),
	RAM_TBL_IDX(OS_ThreadSuspendScheduler),
	RAM_TBL_IDX(OS_ThreadResumeScheduler),
	RAM_TBL_IDX(OS_ThreadIsSchedulerRunning),

	RAM_TBL_IDX(OS_TimerCreate),
	RAM_TBL_IDX(OS_TimerDelete),
	RAM_TBL_IDX(OS_TimerStart),
	RAM_TBL_IDX(OS_TimerChangePeriod),
	RAM_TBL_IDX(OS_TimerStop),
	RAM_TBL_IDX(OS_TimerIsActive),

	RAM_TBL_IDX(OS_HZ),
	RAM_TBL_IDX(OS_GetTicks),
	RAM_TBL_IDX(OS_MSleep),

	RAM_TBL_IDX(HAL_UDelay),
	RAM_TBL_IDX(HAL_Alive),

	/* image */
	RAM_TBL_IDX(flash_rw),
	RAM_TBL_IDX(flash_get_erase_block),
	RAM_TBL_IDX(flash_erase),
	RAM_TBL_IDX(fdcm_open),
	RAM_TBL_IDX(fdcm_read),
	RAM_TBL_IDX(fdcm_write),
	RAM_TBL_IDX(image_checksum16),

	/* flash */
	RAM_TBL_IDX(HAL_Flash_Open),
	RAM_TBL_IDX(HAL_Flash_Close),
	RAM_TBL_IDX(HAL_Flash_Write),
	RAM_TBL_IDX(HAL_Flash_Read),
	RAM_TBL_IDX(HAL_Flash_Erase),
	RAM_TBL_IDX(HAL_Flash_MemoryOf),
	RAM_TBL_IDX(flashcDriverCreate),
	RAM_TBL_IDX(spiDriverCreate),

	RAM_TBL_IDX(FlashChipGetCfgList),
	RAM_TBL_IDX(FlashChipGetChipList),
	RAM_TBL_IDX(defaultControl),
	RAM_TBL_IDX(HAL_Flashc_Delay),
	RAM_TBL_IDX(HAL_Flashc_Xip_Init),
	RAM_TBL_IDX(HAL_Flashc_Xip_Deinit),
	RAM_TBL_IDX(HAL_Flashc_Init),
	RAM_TBL_IDX(HAL_Flashc_Deinit),
	RAM_TBL_IDX(HAL_Flashc_Open),
	RAM_TBL_IDX(HAL_Flashc_Close),
	RAM_TBL_IDX(FC_GetDataDelay),
	RAM_TBL_IDX(FLASH_ICACHE_START_ADDR),
	RAM_TBL_IDX(FLASH_ICACHE_END_ADDR),
	RAM_TBL_IDX(IDCACHE_START_ADDR),
	RAM_TBL_IDX(IDCACHE_END_ADDR),

	/* sdio */
	RAM_TBL_IDX(HAL_SDC_Open),
	RAM_TBL_IDX(HAL_SDC_Close),
	RAM_TBL_IDX(HAL_SDC_Update_Clk),
	RAM_TBL_IDX(HAL_SDC_Clk_PWR_Opt),
	RAM_TBL_IDX(HAL_SDC_PowerOn),
	RAM_TBL_IDX(HAL_SDC_PowerOff),
	RAM_TBL_IDX(HAL_SDC_Request),

	/* pm */
	RAM_TBL_IDX(WakeIo_To_Gpio),
	RAM_TBL_IDX(pm_check_wakeup_irqs),
};

#endif /* __CONFIG_ROM */
#endif /*_ROM_RAM_TABLE_H_*/
