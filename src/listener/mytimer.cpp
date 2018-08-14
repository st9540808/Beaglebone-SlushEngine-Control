#include "mytimer.h"

static struct timespec start, end;

void mytimer_start(void)
{
    clock_gettime(CLOCK_MONOTONIC, &start);
}

void mytimer_end(void)
{
    clock_gettime(CLOCK_MONOTONIC, &end);
}

static double diff(struct timespec start, struct timespec end)
{
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec) < 0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;            
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;            
    }
    return temp.tv_sec + (double) temp.tv_nsec / 1000000000.0;
}

double mytimer_diff(void)
{
    return diff(start, end);
}

