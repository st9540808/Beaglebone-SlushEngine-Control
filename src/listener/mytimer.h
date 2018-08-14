#ifndef MYTIMER_H
#define MYTIMER_H

extern "C" { 
    #include <unistd.h>
    #include <time.h>
}

void mytimer_start(void);
void mytimer_end(void);
double mytimer_diff(void);

#endif

