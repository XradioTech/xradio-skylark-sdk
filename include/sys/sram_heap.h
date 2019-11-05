#ifndef _SRAM_HEAP_H_
#define _SRAM_HEAP_H_
#include <sys/reent.h>

#ifdef __CONFIG_MIX_HEAP_MANAGE
void *__real__malloc_r(struct _reent *reent, size_t size);
void *__real__realloc_r(struct _reent *reent, void *ptr, size_t size);
void __real__free_r(struct _reent *reent, void *ptr);
void *__real_malloc(size_t size);
void *__real_calloc(size_t nmemb, size_t size);
void *__real_realloc(void *ptr, size_t size);
void __real_free(void *ptr);

#define sram_malloc         __real_malloc
#define sram_calloc         __real_calloc
#define sram_realloc        __real_realloc
#define sram_free           __real_free
#define sram_malloc_r       __real__malloc_r
#define sram_realloc_r      __real__realloc_r
#define sram_free_r         __real__free_r
#endif /*__CONFIG_MIX_HEAP_MANAGE*/

#endif /*_SRAM_HEAP_H_*/
