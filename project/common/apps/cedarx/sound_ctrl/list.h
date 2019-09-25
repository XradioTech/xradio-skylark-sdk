/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : list.h
 * Description : list
 * History :
 *
 */

#ifndef _APPS_LIST_H
#define _APPS_LIST_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct ListNodeS ListNodeT;
typedef struct ListS ListT;

struct ListNodeS
{
    struct ListNodeS *next;
    struct ListNodeS *prev;
};

struct ListS
{
    struct ListNodeS *head;
    struct ListNodeS *tail;
};

static void __ListAdd(struct ListNodeS *new,
                        struct ListNodeS *prev, struct ListNodeS *next)
{
    next->prev = new;
    new->next = next;
    new->prev = prev;
    prev->next = new;
}

static inline void ListAddTail(struct ListNodeS *new, struct ListS *list)
{
    __ListAdd(new, list->tail, (struct ListNodeS *)list);
}

/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */
#define ListInit(list) do { \
    (list)->head = (list)->tail = (struct ListNodeS *)(list);\
    }while (0)

#define ListNodeInit(node) do { \
    (node)->next = (node)->prev = (node);\
    }while (0)

#define Offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#define ContainerOf(ptr, type, member) ({ \
    const typeof(((type *)0)->member) *__mptr = (ptr); \
    (type *)((char *)__mptr - Offsetof(type,member) ); })

/**
 * list_entry - get the struct for this entry
 * @ptr:    the &struct list_head pointer.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_struct within the struct.
 */
#define ListEntry(ptr, type, member) \
    ContainerOf(ptr, type, member)

/**
 * list_first_entry - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_struct within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define ListFirstEntry(ptr, type, member) \
    ListEntry((ptr)->head, type, member)

/**
 * list_for_each    -    iterate over a list
 * @pos:    the &struct list_head to use as a loop cursor.
 * @head:    the head for your list.
 */
#define ListForEach(pos, list) \
    for (pos = (list)->head; \
            pos != (struct ListNodeS *)(list);\
            pos = pos->next)

/**
 * list_for_each_prev    -    iterate over a list backwards
 * @pos:    the &struct list_head to use as a loop cursor.
 * @head:    the head for your list.
 */
#define ListForEachPrev(pos, list) \
    for (pos = (list)->tail; \
        pos != (struct ListNodeS *)(list); \
        pos = pos->prev)

/**
 * list_for_each_safe - iterate over a list safe against removal of list entry
 * @pos:    the &struct list_head to use as a loop cursor.
 * @n:        another &struct list_head to use as temporary storage
 * @head:    the head for your list.
 */
#define ListForEachSafe(pos, n, list) \
    for (pos = (list)->head, n = pos->next; \
        pos != (struct ListNodeS *)(list); \
        pos = n, n = pos->next)

/**
 * list_for_each_prev_safe - iterate over a list backwards safe against removal of list entry
 * @pos:    the &struct list_head to use as a loop cursor.
 * @n:        another &struct list_head to use as temporary storage
 * @head:    the head for your list.
 */
#define ListForEachPrevSafe(pos, n, list) \
    for (pos = (list)->tail, n = pos->prev; \
         pos != (struct ListNodeS *)(list); \
         pos = n, n = pos->prev)

/**
 * list_for_each_entry    -    iterate over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the list_struct within the struct.
 */
#define ListForEachEntry(pos, list, member)                \
    for (pos = ListEntry((list)->head, typeof(*pos), member);    \
         &pos->member != (struct ListNodeS *)(list);     \
         pos = ListEntry(pos->member.next, typeof(*pos), member))

/**
 * list_for_each_entry_reverse - iterate backwards over list of given type.
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the list_struct within the struct.
 */
#define ListForEachEntryReverse(pos, list, member)            \
    for (pos = ListEntry((list)->tail, typeof(*pos), member);    \
         &pos->member != (struct ListNodeS *)(list);     \
         pos = ListEntry(pos->member.prev, typeof(*pos), member))

/**
 * list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the list_struct within the struct.
 */
#define ListForEachEntrySafe(pos, n, list, member)            \
    for (pos = ListEntry((list)->head, typeof(*pos), member),    \
        n = ListEntry(pos->member.next, typeof(*pos), member);    \
         &pos->member != (struct ListNodeS *)(list);                     \
         pos = n, n = ListEntry(n->member.next, typeof(*n), member))

/**
 * list_for_each_entry_safe_reverse - iterate backwards over list safe against removal
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the list_struct within the struct.
 *
 * Iterate backwards over list of given type, safe against removal
 * of list entry.
 */
#define ListForEachEntrySafeReverse(pos, n, list, member)        \
    for (pos = ListEntry((list)->prev, typeof(*pos), member),    \
        n = ListEntry(pos->member.prev, typeof(*pos), member);    \
         &pos->member != (struct ListNodeS *)(list);                     \
         pos = n, n = ListEntry(n->member.prev, typeof(*n), member))

#ifdef __cplusplus
}
#endif

#endif