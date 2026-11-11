#include "mem_alloc.h"

#include <stdlib.h>
#include <string.h>

void *zero_malloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}
