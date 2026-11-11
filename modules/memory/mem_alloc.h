#ifndef MEM_ALLOC_H
#define MEM_ALLOC_H

#include <stddef.h>

/* 分配并清零 size 字节。 */
void *zero_malloc(size_t size);

#endif
