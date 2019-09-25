/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */

#ifdef __CONFIG_PSRAM

#include <stdlib.h>
#include <string.h>
/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "driver/chip/hal_dcache.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Block sizes must not get too small. */
#define psram_heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( psram_xHeapStructSize << 1 ) )

/* alligh 16 Bytes */
#define psram_portBYTE_ALIGNMENT		16
#if psram_portBYTE_ALIGNMENT == 32
	#define psram_portBYTE_ALIGNMENT_MASK   ( 0x001f )
#endif
#if psram_portBYTE_ALIGNMENT == 16
	#define psram_portBYTE_ALIGNMENT_MASK   ( 0x000f )
#endif
#if psram_portBYTE_ALIGNMENT == 8
	#define psram_portBYTE_ALIGNMENT_MASK   ( 0x0007 )
#endif

/* Assumes 8bit bytes! */
#define psram_heapBITS_PER_BYTE			( ( size_t ) 8 )

/* Allocate the memory for the heap. */
	extern uint8_t __psram_end__[];	/* heap start address defined by linker */

	#define psram_configTOTAL_HEAP_SIZE	( IDCACHE_END_ADDR - (size_t)__psram_end__ )

	static uint8_t *psram_ucHeap = __psram_end__;

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
} psram_BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void psram_prvInsertBlockIntoFreeList( psram_BlockLink_t *pxBlockToInsert );

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void psram_heap_init( void );

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const size_t psram_xHeapStructSize	= ( sizeof( psram_BlockLink_t ) + ( ( size_t ) ( psram_portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) psram_portBYTE_ALIGNMENT_MASK );

/* Create a couple of list links to mark the start and end of the list. */
static psram_BlockLink_t psram_xStart, *psram_pxEnd = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t psram_xFreeBytesRemaining = 0U;
static size_t psram_xMinimumEverFreeBytesRemaining = 0U;

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
member of an psram_BlockLink_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static size_t psram_xBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

void *psram_malloc( size_t xWantedSize )
{
psram_BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( psram_pxEnd == NULL )
		{
			psram_heap_init();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top bit is
		set.  The top bit of the block size member of the psram_BlockLink_t structure
		is used to determine who owns the block - the application or the
		kernel, so it must be free. */
		if( ( xWantedSize & psram_xBlockAllocatedBit ) == 0 )
		{
			/* The wanted size is increased so it can contain a psram_BlockLink_t
			structure in addition to the requested amount of bytes. */
			if( xWantedSize > 0 )
			{
				xWantedSize += psram_xHeapStructSize;

				/* Ensure that blocks are always aligned to the required number
				of bytes. */
				if( ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) != 0x00 )
				{
					/* Byte alignment required. */
					xWantedSize += ( psram_portBYTE_ALIGNMENT - ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) );
					configASSERT( ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) == 0 );
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= psram_xFreeBytesRemaining ) )
			{
				/* Traverse the list from the start	(lowest address) block until
				one	of adequate size is found. */
				pxPreviousBlock = &psram_xStart;
				pxBlock = psram_xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				was	not found. */
				if( pxBlock != psram_pxEnd )
				{
					/* Return the memory space pointed to - jumping over the
					psram_BlockLink_t structure at its start. */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + psram_xHeapStructSize );

					/* This block is being returned for use so must be taken out
					of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					two. */
					if( ( pxBlock->xBlockSize - xWantedSize ) > psram_heapMINIMUM_BLOCK_SIZE )
					{
						/* This block is to be split into two.  Create a new
						block following the number of bytes requested. The void
						cast is used to prevent byte alignment warnings from the
						compiler. */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						configASSERT( ( ( ( size_t ) pxNewBlockLink ) & psram_portBYTE_ALIGNMENT_MASK ) == 0 );

						/* Calculate the sizes of two blocks split from the
						single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						psram_prvInsertBlockIntoFreeList( pxNewBlockLink );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					psram_xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( psram_xFreeBytesRemaining < psram_xMinimumEverFreeBytesRemaining )
					{
						psram_xMinimumEverFreeBytesRemaining = psram_xFreeBytesRemaining;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					by the application and has no "next" block. */
					pxBlock->xBlockSize |= psram_xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	configASSERT( ( ( ( uint32_t ) pvReturn ) & psram_portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}
/*-----------------------------------------------------------*/

void psram_free( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
psram_BlockLink_t *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an psram_BlockLink_t structure immediately
		before it. */
		puc -= psram_xHeapStructSize;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		/* Check the block is actually allocated. */
		configASSERT( ( pxLink->xBlockSize & psram_xBlockAllocatedBit ) != 0 );
		configASSERT( pxLink->pxNextFreeBlock == NULL );

		if( ( pxLink->xBlockSize & psram_xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				/* The block is being returned to the heap - it is no longer
				allocated. */
				pxLink->xBlockSize &= ~psram_xBlockAllocatedBit;

				vTaskSuspendAll();
				{
					/* Add this block to the list of free blocks. */
					psram_xFreeBytesRemaining += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					psram_prvInsertBlockIntoFreeList( ( ( psram_BlockLink_t * ) pxLink ) );
				}
				( void ) xTaskResumeAll();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
}
/*-----------------------------------------------------------*/

size_t psram_GetFreeHeapSize( void )
{
	return psram_xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t psram_GetMinimumEverFreeHeapSize( void )
{
	return psram_xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

static void psram_heap_init( void )
{
psram_BlockLink_t *pxFirstFreeBlock;
uint8_t *pucAlignedHeap;
size_t uxAddress;
size_t xTotalHeapSize = psram_configTOTAL_HEAP_SIZE;

	/* Ensure the heap starts on a correctly aligned boundary. */
	uxAddress = ( size_t ) psram_ucHeap;

	if( ( uxAddress & psram_portBYTE_ALIGNMENT_MASK ) != 0 )
	{
		uxAddress += ( psram_portBYTE_ALIGNMENT - 1 );
		uxAddress &= ~( ( size_t ) psram_portBYTE_ALIGNMENT_MASK );
		xTotalHeapSize -= uxAddress - ( size_t ) psram_ucHeap;
	}

	pucAlignedHeap = ( uint8_t * ) uxAddress;

	/* psram_xStart is used to hold a pointer to the first item in the list of free
	blocks.  The void cast is used to prevent compiler warnings. */
	psram_xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	psram_xStart.xBlockSize = ( size_t ) 0;

	/* psram_pxEnd is used to mark the end of the list of free blocks and is inserted
	at the end of the heap space. */
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= psram_xHeapStructSize;
	uxAddress &= ~( ( size_t ) psram_portBYTE_ALIGNMENT_MASK );
	psram_pxEnd = ( void * ) uxAddress;
	psram_pxEnd->xBlockSize = 0;
	psram_pxEnd->pxNextFreeBlock = NULL;

	/* To start with there is a single free block that is sized to take up the
	entire heap space, minus the space taken by psram_pxEnd. */
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = psram_pxEnd;

	/* Only one block exists - and it covers the entire usable heap space. */
	psram_xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	psram_xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

	/* Work out the position of the top bit in a size_t variable. */
	psram_xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * psram_heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/

static void psram_prvInsertBlockIntoFreeList( psram_BlockLink_t *pxBlockToInsert )
{
psram_BlockLink_t *pxIterator;
uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &psram_xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != psram_pxEnd )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = psram_pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's pxNextFreeBlock pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}

void *psram_realloc( uint8_t *srcaddr,size_t xWantedSize )
{
	psram_BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
	void *pvReturn = NULL;

	psram_BlockLink_t *pxBlockold,*pxBlockjudge;
	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( psram_pxEnd == NULL )
		{
			psram_heap_init();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top bit is
		set.  The top bit of the block size member of the BlockLink_t structure
		is used to determine who owns the block - the application or the
		kernel, so it must be free. */
		if( ( xWantedSize & psram_xBlockAllocatedBit ) == 0 )
		{
			if( ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) != 0x00 )
			{
				/* Byte alignment required. */
				xWantedSize += ( psram_portBYTE_ALIGNMENT - ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) );
				configASSERT( ( xWantedSize & psram_portBYTE_ALIGNMENT_MASK ) == 0 );
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= psram_xFreeBytesRemaining ) )
			{
				if(srcaddr == NULL)
				{
					pvReturn = psram_malloc(xWantedSize);
					goto out;
				}
                pxBlockold = (psram_BlockLink_t *)(srcaddr -  psram_xHeapStructSize);
				pxBlockjudge = (psram_BlockLink_t *)((uint8_t*)pxBlockold+((pxBlockold->xBlockSize)&(~psram_xBlockAllocatedBit)));

				pxPreviousBlock = &psram_xStart;
				pxBlock = psram_xStart.pxNextFreeBlock;
				while(pxBlock != pxBlockjudge&& ( pxBlock->pxNextFreeBlock != NULL ))
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}
				if((xWantedSize< pxBlock->xBlockSize&&(( pxBlock->xBlockSize - xWantedSize ) > psram_heapMINIMUM_BLOCK_SIZE))&&pxBlock == pxBlockjudge)
				{
					pxBlockold->xBlockSize += xWantedSize;
					pxNewBlockLink = (psram_BlockLink_t *)((uint8_t*)pxBlock + xWantedSize);
					pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;

					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;
						/* Insert the new block into the list of free blocks. */
					psram_prvInsertBlockIntoFreeList( pxNewBlockLink );
					pxBlock->pxNextFreeBlock = NULL;
					pvReturn = srcaddr;
					psram_xFreeBytesRemaining -= xWantedSize;

					if( psram_xFreeBytesRemaining < psram_xMinimumEverFreeBytesRemaining )
					{
						psram_xMinimumEverFreeBytesRemaining = psram_xFreeBytesRemaining;
					}
				}
				else
				{
				    pvReturn = psram_malloc((((pxBlockold->xBlockSize)&(~psram_xBlockAllocatedBit))-psram_xHeapStructSize)+xWantedSize);
					memcpy((uint8_t*)pvReturn,srcaddr,((pxBlockold->xBlockSize&(~psram_xBlockAllocatedBit))-psram_xHeapStructSize));
					psram_free(srcaddr);
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
out:
	( void ) xTaskResumeAll();
	configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) psram_portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}

void *psram_calloc( size_t xNmemb, size_t xMembSize )
{
    void *ptr = psram_malloc(xNmemb*xMembSize);
    if(ptr != NULL) {
        memset(ptr, 0, xNmemb*xMembSize);
    }

    return ptr;
}

#endif /* __CONFIG_PSRAM */
