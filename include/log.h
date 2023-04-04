#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <stdlib.h>

#define MY_LOG(...) { \
    printf("[LOG]: %s:%s %d\n", __FILE__, __FUNCTION__, __LINE__); \
}

#endif
