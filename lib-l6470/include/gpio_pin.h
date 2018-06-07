/**
 * @file gpio_pin.h
 *
 */
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#define FATAL(str)                                             \
    do {                                                       \
        fprintf(stderr, "Error at line %d, file %s (%d)\n",    \
                __LINE__, __FILE__, errno);                    \
        perror(str); exit(1);                                  \
    } while (0)

#ifdef DEBUG
 #define DEBUG_PRINT(fmt, args...) fprintf(stderr, \
    "DEBUG: %s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##args)
#else
 #define DEBUG_PRINT(fmt, args...) do {} while (0)
#endif

#define CM_PER (0x44E00000)
#define CM_PER_GPIO2_CLKCTRL (0xB0)
#define CM_PER_GPIO3_CLKCTRL (0xB4)


#define GPIO0 	(0x44E07000)
#define GPIO1	(0x4804C000)
#define GPIO2	(0x481AC000)
#define GPIO3	(0x481AE000)
#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT   (0x194)
#define GPIO_OE	          (0x134)
#define GPIO_DATAOUT      (0x13C)
#define GPIO_DATAIN       (0x138)

#define P8_7  ((PIN){   "TIMER4",      GPIO2,     66, 	2})
#define P8_8  ((PIN){   "TIMER7",      GPIO2,     67, 	3})
#define P8_9  ((PIN){   "TIMER5",      GPIO2,     69, 	5})
#define P8_10 ((PIN){   "TIMER6",      GPIO2,     68, 	4})
#define P8_11 ((PIN){   "GPIO1_13",    GPIO1,     45, 	13})
#define P8_12 ((PIN){   "GPIO1_12",    GPIO1,     44, 	12})

struct PIN {
    const char* name;   /*!< readable name of pin, i.e.: "GPIO1_21", see beaglebone user guide */
    uint32_t gpio_bank; /*!< which of the four gpio banks is this pin in, i.e.: GPIO1, r 0x4804C000 */
    uint8_t gpio;       /*!< pin number on the am335x processor */
    uint8_t bank_id;    /*!< pin number within each bank, should be 0-31 */
};

class GPIO_Pin {
public:
    GPIO_Pin()
        : MAP_SIZE(sysconf(_SC_PAGESIZE))
        , MAP_MASK(MAP_SIZE - 1)
    {}

    void init(PIN p) {
        int fd;
        
        this->bank = p.bank_id;

        if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
            FATAL("failed open /dev/mem");
        
        if (!CM_PER_base) {
            CM_PER_base = (uint8_t *)
                mmap(0, 1024, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, CM_PER & ~MAP_MASK);
            if (CM_PER_base == (uint8_t *) -1)
                FATAL("failed mmap on CM_PER");
        }

        // check if module is enabled
        switch (p.gpio_bank) {
        case GPIO2: 
            if ((*(uint32_t *)(CM_PER_base + CM_PER_GPIO2_CLKCTRL) & 0x3) == 0)
                 *(uint32_t *)(CM_PER_base + CM_PER_GPIO2_CLKCTRL) = 0x2;
            DEBUG_PRINT("%#010x\n",
                        *(uint32_t *)(CM_PER_base + CM_PER_GPIO2_CLKCTRL));
            
            if (!GPIO2_base) {
                GPIO2_base = (uint8_t *)
                    mmap(0, 4096, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, GPIO2 & ~MAP_MASK);
                if (GPIO2_base == (uint8_t *) -1)
                    FATAL("failed mmap on CM_PER");

                if (mlock((void *) GPIO2_base, 4096) == -1)
                    FATAL("failed mlock");
            }

            // set direction to output for GPIO pin
            *(uint32_t *)(GPIO2_base + GPIO_OE) &= ~(1 << this->bank);
            DATAOUT_addr = GPIO2_base + GPIO_DATAOUT;
            SETDATAOUT_addr = GPIO2_base + GPIO_SETDATAOUT;
            CLEARDATAOUT_addr = GPIO2_base + GPIO_CLEARDATAOUT;
            break;
        
        case GPIO1:
            if (!GPIO1_base) {
                GPIO1_base = (uint8_t *)
                    mmap(0, 4096, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, GPIO1 & ~MAP_MASK);
                if (GPIO1_base == (uint8_t *) -1)
                    FATAL("failed mmap on CM_PER");

                if (mlock((void *) GPIO1_base, 4096) == -1)
                    FATAL("failed mlock");
            }

            // set direction to output for GPIO pin
            *(uint32_t *)(GPIO1_base + GPIO_OE) &= ~(1 << this->bank);
            DATAOUT_addr = GPIO1_base + GPIO_DATAOUT;
            SETDATAOUT_addr = GPIO1_base + GPIO_SETDATAOUT;
            CLEARDATAOUT_addr = GPIO1_base + GPIO_CLEARDATAOUT;
            break;

        default:
            FATAL("wrong PIN");
        }

        close(fd);
    }

    void set(void) {
        *(uint32_t *)(SETDATAOUT_addr) = 1 << this->bank;
    }

    void clear(void) {
        *(uint32_t *)(CLEARDATAOUT_addr) = 1 << this->bank;
    }

private:
    // GPIO_Pin global values
    const unsigned long MAP_SIZE;
    const unsigned long MAP_MASK;
    static volatile uint8_t* CM_PER_base;
    static volatile uint8_t* GPIO2_base;
    static volatile uint8_t* GPIO1_base;

    uint8_t bank;
    volatile uint8_t* DATAOUT_addr;
    volatile uint8_t* SETDATAOUT_addr;
    volatile uint8_t* CLEARDATAOUT_addr;
};

volatile uint8_t* GPIO_Pin::CM_PER_base = nullptr;
volatile uint8_t* GPIO_Pin::GPIO2_base = nullptr;
volatile uint8_t* GPIO_Pin::GPIO1_base = nullptr;
