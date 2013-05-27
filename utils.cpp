#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#include <signal.h>
#include <execinfo.h>
#include <errno.h>
#include <cxxabi.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include <dirent.h>

#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <math.h>
#include <complex.h>
#include <float.h>

////////////////////////////////////////////////////////////////////////////////
// time functions
////////////////////////////////////////////////////////////////////////////////

u_int64_t tm_get_millis(void)
{
    struct timeval  tm_val;
    u_int64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;
    return v;
}

u_int64_t tm_get_ms(void)
{
    struct timeval  tm_val;
    u_int64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;
    return v;
}

u_int64_t tm_get_us(void)
{
    struct timeval  tm_val;
    u_int64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000000 + tm_val.tv_usec;
    return v;
}

void   tm_sleep(u_int32_t t)
{
    struct timespec tp;

    tp.tv_sec = t / 1000;
    tp.tv_nsec = ( t % 1000 ) * 1000000;

    nanosleep(&tp, &tp);
}
