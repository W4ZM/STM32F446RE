#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

void usart_init(void);
void usart_wait_for_flag(uint32_t mask, bool flag);
void usart_send(const uint8_t* data, uint8_t size);

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

int _isatty(int fd) {

  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
  {
    usart_send((uint8_t *) ptr, len);
    return len;
  }

  errno = EBADF;
  return -1;
}

int _close(int fd) {

  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {

  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {

  if (fd == STDIN_FILENO) return 1;
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {

  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}

void* _sbrk(ptrdiff_t incr){

  // consider as heapoverflow
  errno = ENOMEM;
  return (void*) -1;
}
