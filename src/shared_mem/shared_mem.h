#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_
#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <sys/types.h>
#include <unistd.h>

// for C
#ifdef __cplusplus
extern "C"
{
#endif

    // get shared memory virtual address with specific name and size in byte.
    // void *get_shared_memory(void);
    void *get_shared_memory(const char *shared_mem_name, const char *semaphore_name, const int shared_mem_size);

    // void release_shared_memory(const char *shared_mem_name,
    //                            const int shared_mem_size);

#ifdef __cplusplus
}
#endif

#endif