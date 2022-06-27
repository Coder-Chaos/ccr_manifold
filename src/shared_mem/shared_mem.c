#include "shared_mem.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // #define SHM_SEGMENT_NAME "/demo-shm"

#define USE_SEM 1

#if USE_SEM
#define SEMA_NAME "/demo-sem"
  static sem_t *sem_p;
#endif

  static int shm_fd;
  static struct shared_data *shm_p;
  /*
 * If the shared memory segment does not exist already, create it
 * Returns a pointer to the segment or NULL if there is an error
 */
  void *get_shared_memory(const char *shared_mem_name, const char *semaphore_name, const int shared_mem_size)
  {
    /* Attempt to create the shared memory segment */
    shm_fd = shm_open(shared_mem_name, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (shm_fd > 0)
    {
      /* succeeded: expand it to the desired size (Note: dont't do
    "this every time because ftruncate fills it with zeros) */
      printf("Creating shared memory and setting size=%d\n", shared_mem_size);
      if (ftruncate(shm_fd, shared_mem_size) < 0)
      {
        perror("ftruncate");
        exit(1);
      }
#if USE_SEM
      /* Create a semaphore as well */
      sem_p = sem_open(semaphore_name, O_RDWR | O_CREAT, 0666, 1);
      if (sem_p == SEM_FAILED)
        perror("sem_open failed\n");
#endif
    }
    else if (shm_fd == -1 && errno == EEXIST)
    {
      /* Already exists: open again without O_CREAT */
      shm_fd = shm_open(shared_mem_name, O_RDWR, 0);
#if USE_SEM
      sem_p = sem_open(semaphore_name, O_RDWR);
      if (sem_p == SEM_FAILED)
        perror("sem_open failed\n");
#endif
    }
    if (shm_fd == -1)
    {
      perror("shm_open ");
      exit(1);
    }
    /* Map the shared memory */
    shm_p = mmap(NULL, shared_mem_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                 shm_fd, 0);
    if (shm_p == NULL)
    {
      perror("mmap");
      exit(1);
    }

    // get semaphore
    // sem=(sem_t **)&sem_p;
    return shm_p;
  }

  // release shared memory

#ifdef __cplusplus
}
#endif