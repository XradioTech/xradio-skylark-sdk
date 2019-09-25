#ifndef _SYS_HEAP_H_
#define _SYS_HEAP_H_
/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
    size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

typedef struct sys_heap
{
    size_t heapBits_Per_Byte;
    uint8_t portByte_Alignment;
    uint8_t portByte_Alignment_Mask;
    size_t configTotal_Heap_Size;
    size_t heapMinmun_Block_Size;
    size_t xHeapStructSize;
    uint8_t *ucHeap;
    BlockLink_t xStart;
    BlockLink_t *pxEnd;
    size_t xFreeBytesRemaining;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xBlockAllocatedBit;
} sys_heap_t;

#define SYSHEAP_DEFAULT_INIT(sysHeap, baseAddr, total_size) (sysHeap)->heapBits_Per_Byte = ( size_t ) 8; \
    (sysHeap)->portByte_Alignment = 8; \
    (sysHeap)->portByte_Alignment_Mask = 0x7; \
    (sysHeap)->xHeapStructSize = ( sizeof( BlockLink_t ) + ( ( size_t ) ( (sysHeap)->portByte_Alignment - 1 ) ) ) & ~( ( size_t ) (sysHeap)->portByte_Alignment_Mask ); \
    (sysHeap)->heapMinmun_Block_Size = ( ( size_t ) ( (sysHeap)->xHeapStructSize << 1 ) ); \
    (sysHeap)->xFreeBytesRemaining = 0; \
    (sysHeap)->xMinimumEverFreeBytesRemaining = 0; \
    (sysHeap)->xBlockAllocatedBit = 0; \
    (sysHeap)->pxEnd = NULL; \
    (sysHeap)->configTotal_Heap_Size = total_size; \
    (sysHeap)->ucHeap = baseAddr;

void sys_heap_free( sys_heap_t *sysHeap, void *ptr );
void *sys_heap_malloc( sys_heap_t *sysHeap, size_t size );
void *sys_heap_realloc( sys_heap_t *sysHeap, uint8_t *ptr,size_t size );
void *sys_heap_calloc( sys_heap_t *sysHeap, size_t nmemb, size_t size );

int sys_heap_init( sys_heap_t *sysHeap );

#endif /*_SYS_HEAP_H_*/
