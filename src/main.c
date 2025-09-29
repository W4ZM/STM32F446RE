#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>
#include <errno.h>
#include "stm32f4xx.h"


void usart_init(void);
void usart_wait_for_flag(uint32_t mask, bool flag);
void usart_send(const uint8_t* data, uint8_t size);

int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _fstat(int fd, struct stat* st);
int _read(int fd, char* ptr, int len);
void* _sbrk(ptrdiff_t incr);

void main(void){
  
  usart_init();

  // disable I/O buffering for STDOUT stream
  setvbuf(stdout, NULL, _IONBF, 0);

  uint count = 0;
  while (1)
  {
    for (uint32_t i = 0; i < 2000000; i++);
    printf(" %d", count);
    count++;
  }
}