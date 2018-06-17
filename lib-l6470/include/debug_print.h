#include <stdio.h>
#include <errno.h>

#ifndef FATAL
 #define FATAL(str)                                             \
     do {                                                       \
         fprintf(stderr, str "errno %d, file %s (%d)\n",        \
                 errno, __FILE__, __LINE__);                    \
         exit(1);                                               \
     } while (0)
#endif

#ifndef DEBUG_PRINT
 #ifdef DEBUG
  #define DEBUG_PRINT(fmt, args...) fprintf(stderr, \
     "DEBUG: %s(): " fmt "\n" , __func__, ##args)
 #else
  #define DEBUG_PRINT(fmt, args...) do {} while (0)
 #endif
#endif
