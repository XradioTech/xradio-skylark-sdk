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

#include <stdlib.h>
#include <string.h>
/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "sys/sys_heap.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void sys_heap_prvInsertBlockIntoFreeList( sys_heap_t *sysHeap, BlockLink_t *pxBlockToInsert );

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void *sys_heap_malloc( sys_heap_t *sysHeap, size_t size )
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    vTaskSuspendAll();
    {
        /* If this is the first call to malloc then the heap will require
        initialisation to setup the list of free blocks. */
        if( sysHeap->pxEnd == NULL ) {
            goto out;
        } else {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Check the requested block size is not so large that the top bit is
        set.  The top bit of the block size member of the BlockLink_t structure
        is used to determine who owns the block - the application or the
        kernel, so it must be free. */
        if( ( size & sysHeap->xBlockAllocatedBit ) == 0 ) {
            /* The wanted size is increased so it can contain a BlockLink_t
            structure in addition to the requested amount of bytes. */
            if( size > 0 ) {
                size += sysHeap->xHeapStructSize;

                /* Ensure that blocks are always aligned to the required number
                of bytes. */
                if( ( size & sysHeap->portByte_Alignment_Mask ) != 0x00 ) {
                    /* Byte alignment required. */
                    size += ( sysHeap->portByte_Alignment - ( size & sysHeap->portByte_Alignment_Mask ) );
                    configASSERT( ( size & sysHeap->portByte_Alignment_Mask ) == 0 );
                } else {
                    mtCOVERAGE_TEST_MARKER();
                }
            } else {
                mtCOVERAGE_TEST_MARKER();
            }

            if( ( size > 0 ) && ( size <= sysHeap->xFreeBytesRemaining ) ) {
                /* Traverse the list from the start	(lowest address) block until
                one	of adequate size is found. */
                pxPreviousBlock = &sysHeap->xStart;
                pxBlock = sysHeap->xStart.pxNextFreeBlock;
                while( ( pxBlock->xBlockSize < size ) && ( pxBlock->pxNextFreeBlock != NULL ) ) {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }

                /* If the end marker was reached then a block of adequate size
                was	not found. */
                if( pxBlock != sysHeap->pxEnd ) {
                    /* Return the memory space pointed to - jumping over the
                    BlockLink_t structure at its start. */
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + sysHeap->xHeapStructSize );

                    /* This block is being returned for use so must be taken out
                    of the list of free blocks. */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    /* If the block is larger than required it can be split into
                    two. */
                    if( ( pxBlock->xBlockSize - size ) > sysHeap->heapMinmun_Block_Size ) {
                        /* This block is to be split into two.  Create a new
                        block following the number of bytes requested. The void
                        cast is used to prevent byte alignment warnings from the
                        compiler. */
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + size );
                        configASSERT( ( ( ( size_t ) pxNewBlockLink ) & sysHeap->portByte_Alignment_Mask ) == 0 );

                        /* Calculate the sizes of two blocks split from the
                        single block. */
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - size;
                        pxBlock->xBlockSize = size;

                        /* Insert the new block into the list of free blocks. */
                        sys_heap_prvInsertBlockIntoFreeList( sysHeap, pxNewBlockLink );
                    } else {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    sysHeap->xFreeBytesRemaining -= pxBlock->xBlockSize;

                    if( sysHeap->xFreeBytesRemaining < sysHeap->xMinimumEverFreeBytesRemaining ) {
                        sysHeap->xMinimumEverFreeBytesRemaining = sysHeap->xFreeBytesRemaining;
                    } else {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* The block is being returned - it is allocated and owned
                    by the application and has no "next" block. */
                    pxBlock->xBlockSize |= sysHeap->xBlockAllocatedBit;
                    pxBlock->pxNextFreeBlock = NULL;
                } else {
                    mtCOVERAGE_TEST_MARKER();
                }
            } else {
                mtCOVERAGE_TEST_MARKER();
            }
        } else {
            mtCOVERAGE_TEST_MARKER();
        }

        traceMALLOC( pvReturn, size );
    }

out:
    ( void ) xTaskResumeAll();

#if( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL ) {
            extern void vApplicationMallocFailedHook( void );
            vApplicationMallocFailedHook();
        } else {
            mtCOVERAGE_TEST_MARKER();
        }
    }
#endif

    configASSERT( ( ( ( uint32_t ) pvReturn ) & sysHeap->portByte_Alignment_Mask ) == 0 );
    return pvReturn;
}
/*-----------------------------------------------------------*/

void sys_heap_free( sys_heap_t *sysHeap, void *ptr )
{
    uint8_t *puc = ( uint8_t * ) ptr;
    BlockLink_t *pxLink;

    if( ptr != NULL ) {
        /* The memory being freed will have an BlockLink_t structure immediately
        before it. */
        puc -= sysHeap->xHeapStructSize;

        /* This casting is to keep the compiler from issuing warnings. */
        pxLink = ( void * ) puc;

        /* Check the block is actually allocated. */
        configASSERT( ( pxLink->xBlockSize & sysHeap->xBlockAllocatedBit ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( ( pxLink->xBlockSize & sysHeap->xBlockAllocatedBit ) != 0 ) {
            if( pxLink->pxNextFreeBlock == NULL ) {
                /* The block is being returned to the heap - it is no longer
                allocated. */
                pxLink->xBlockSize &= ~(sysHeap->xBlockAllocatedBit);

                vTaskSuspendAll();
                {
                    /* Add this block to the list of free blocks. */
                    sysHeap->xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( ptr, pxLink->xBlockSize );
                    sys_heap_prvInsertBlockIntoFreeList( sysHeap, ( ( BlockLink_t * ) pxLink ) );
                }
                ( void ) xTaskResumeAll();
            } else {
                mtCOVERAGE_TEST_MARKER();
            }
        } else {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}
/*-----------------------------------------------------------*/

size_t sys_heap_xPortGetFreeHeapSize( sys_heap_t *sysHeap )
{
    return sysHeap->xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t sys_heap_xPortGetMinimumEverFreeHeapSize( sys_heap_t *sysHeap )
{
    return sysHeap->xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void sys_heap_vPortInitialiseBlocks( sys_heap_t *sysHeap )
{
    /* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

int sys_heap_init( sys_heap_t *sysHeap )
{
    BlockLink_t *pxFirstFreeBlock;
    uint8_t *pucAlignedHeap;
    size_t uxAddress;
    size_t xTotalHeapSize = sysHeap->configTotal_Heap_Size;

    if(sysHeap == NULL) return -1;
    /* Ensure the heap starts on a correctly aligned boundary. */
    uxAddress = ( size_t ) sysHeap->ucHeap;

    if( ( uxAddress & sysHeap->portByte_Alignment_Mask ) != 0 ) {
        uxAddress += ( sysHeap->portByte_Alignment - 1 );
        uxAddress &= ~( ( size_t ) sysHeap->portByte_Alignment_Mask );
        xTotalHeapSize -= uxAddress - ( size_t ) sysHeap->ucHeap;
    }

    pucAlignedHeap = ( uint8_t * ) uxAddress;

    /* xStart is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    sysHeap->xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    sysHeap->xStart.xBlockSize = ( size_t ) 0;

    /* pxEnd is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
    uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
    uxAddress -= sysHeap->xHeapStructSize;
    uxAddress &= ~( ( size_t ) sysHeap->portByte_Alignment_Mask );
    sysHeap->pxEnd = ( void * ) uxAddress;
    sysHeap->pxEnd->xBlockSize = 0;
    sysHeap->pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by pxEnd. */
    pxFirstFreeBlock = ( void * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = sysHeap->pxEnd;

    /* Only one block exists - and it covers the entire usable heap space. */
    sysHeap->xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    sysHeap->xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

    /* Work out the position of the top bit in a size_t variable. */
    sysHeap->xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * (sysHeap->heapBits_Per_Byte) ) - 1 );

    return 0;
}
/*-----------------------------------------------------------*/

static void sys_heap_prvInsertBlockIntoFreeList( sys_heap_t *sysHeap, BlockLink_t *pxBlockToInsert )
{
    BlockLink_t *pxIterator;
    uint8_t *puc;

    /* Iterate through the list until a block is found that has a higher address
    than the block being inserted. */
    for( pxIterator = &sysHeap->xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock ) {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
    make a contiguous block of memory? */
    puc = ( uint8_t * ) pxIterator;
    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert ) {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    } else {
        mtCOVERAGE_TEST_MARKER();
    }

    /* Do the block being inserted, and the block it is being inserted before
    make a contiguous block of memory? */
    puc = ( uint8_t * ) pxBlockToInsert;
    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock ) {
        if( pxIterator->pxNextFreeBlock != sysHeap->pxEnd ) {
            /* Form one big block from the two blocks. */
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        } else {
            pxBlockToInsert->pxNextFreeBlock = sysHeap->pxEnd;
        }
    } else {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
    before and the block after, then it's pxNextFreeBlock pointer will have
    already been set, and should not be set here as that would make it point
    to itself. */
    if( pxIterator != pxBlockToInsert ) {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    } else {
        mtCOVERAGE_TEST_MARKER();
    }
}


void *sys_heap_realloc( sys_heap_t *sysHeap, uint8_t *ptr, size_t size )
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    BlockLink_t *pxBlockold,*pxBlockjudge;
    vTaskSuspendAll();
    {
        /* If this is the first call to malloc then the heap will require
        initialisation to setup the list of free blocks. */
        if( sysHeap->pxEnd == NULL ) {
            goto out;
        } else {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Check the requested block size is not so large that the top bit is
        set.  The top bit of the block size member of the BlockLink_t structure
        is used to determine who owns the block - the application or the
        kernel, so it must be free. */
        if( ( size & sysHeap->xBlockAllocatedBit ) == 0 ) {
            if( ( size & sysHeap->portByte_Alignment_Mask ) != 0x00 ) {
                /* Byte alignment required. */
                size += ( sysHeap->portByte_Alignment - ( size & sysHeap->portByte_Alignment_Mask ) );
                configASSERT( ( size & sysHeap->portByte_Alignment_Mask ) == 0 );
            } else {
                mtCOVERAGE_TEST_MARKER();
            }

            if( ( size > 0 ) && ( size <= sysHeap->xFreeBytesRemaining ) ) {
                if(ptr == NULL) {
                    pvReturn = sys_heap_malloc(sysHeap, size);
                    goto out;
                }
                pxBlockold = (BlockLink_t *)(ptr -  sysHeap->xHeapStructSize);
                pxBlockjudge = (BlockLink_t *)((uint8_t*)pxBlockold+((pxBlockold->xBlockSize)&(~sysHeap->xBlockAllocatedBit)));

                pxPreviousBlock = &(sysHeap->xStart);
                pxBlock = (sysHeap->xStart).pxNextFreeBlock;
                while(pxBlock != pxBlockjudge&& ( pxBlock->pxNextFreeBlock != NULL )) {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }
                if((size< pxBlock->xBlockSize&&(( pxBlock->xBlockSize - size ) > (sysHeap->heapMinmun_Block_Size))) && pxBlock == pxBlockjudge) {
                    pxBlockold->xBlockSize += size;
                    pxNewBlockLink = (BlockLink_t *)((uint8_t*)pxBlock + size);
                    pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - size;

                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;
                    /* Insert the new block into the list of free blocks. */
                    sys_heap_prvInsertBlockIntoFreeList(sysHeap, pxNewBlockLink );
                    pxBlock->pxNextFreeBlock = NULL;
                    pvReturn = ptr;
                    sysHeap->xFreeBytesRemaining -= size;

                    if( sysHeap->xFreeBytesRemaining < sysHeap->xMinimumEverFreeBytesRemaining ) {
                        sysHeap->xMinimumEverFreeBytesRemaining = sysHeap->xFreeBytesRemaining;
                    }
                } else {
                    pvReturn = sys_heap_malloc(sysHeap, (((pxBlockold->xBlockSize)&(~(sysHeap->xBlockAllocatedBit)))-(sysHeap->xHeapStructSize))+size);
                    memcpy((uint8_t*)pvReturn,ptr,((pxBlockold->xBlockSize&(~(sysHeap->xBlockAllocatedBit)))-sysHeap->xHeapStructSize));
                    sys_heap_free(sysHeap, ptr);
                }
            } else {
                mtCOVERAGE_TEST_MARKER();
            }
        } else {
            mtCOVERAGE_TEST_MARKER();
        }
    }
out:
    ( void ) xTaskResumeAll();
    configASSERT( ( ( ( size_t ) pvReturn ) & (( size_t ) (sysHeap->portByte_Alignment_Mask)) ) == 0 );
    return pvReturn;
}


void *sys_heap_calloc( sys_heap_t *sysHeap, size_t nmemb, size_t size )
{
    void *ptr = sys_heap_malloc(sysHeap, nmemb*size);
    if(ptr != NULL) {
        memset(ptr, 0, nmemb*size);
    }

    return ptr;
}

