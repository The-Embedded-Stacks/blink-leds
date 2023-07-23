#include <errno.h>
#undef errno
extern int errno;

int _write(int file, char *ptr, int len)
{
    errno = ENOSYS;
    return -1;
}

int _read(int file, char *ptr, int len)
{
    errno = ENOSYS;
    return -1;
}

int _close(int file, int ptr, int dir) 
{
    errno = ENOSYS;
    return -1;
}

int _lseek(int file, int ptr, int dir) 
{
    errno = ENOSYS;
    return -1;
}