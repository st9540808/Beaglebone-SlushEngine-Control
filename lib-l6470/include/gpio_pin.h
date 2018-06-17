/**
 * @file gpio_pin.h
 *
 */
#ifndef _GPIO_PIN_H
#define _GPIO_PIN_H

#include <cstdint>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "debug_print.h"


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

#define P8_7  ((PIN){  "P8_7"   ,"TIMER4",      GPIO2,     66, 	2})
#define P8_8  ((PIN){  "P8_8"   ,"TIMER7",      GPIO2,     67, 	3})
#define P8_9  ((PIN){  "P8_9"   ,"TIMER5",      GPIO2,     69, 	5})
#define P8_10 ((PIN){  "P8_10"  ,"TIMER6",      GPIO2,     68, 	4})
#define P8_11 ((PIN){  "P8_11"  ,"GPIO1_13",    GPIO1,     45, 	13})
#define P8_12 ((PIN){  "P8_12"  ,"GPIO1_12",    GPIO1,     44, 	12})

struct PIN {
    const char* location;
    const char* name;   /*!< readable name of pin, i.e.: "GPIO1_21", see beaglebone user guide */
    uint32_t gpio_bank; /*!< which of the four gpio banks is this pin in, i.e.: GPIO1, r 0x4804C000 */
    uint8_t gpio;       /*!< pin number on the am335x processor */
    uint8_t bank_id;    /*!< pin number within each bank, should be 0-31 */
};

class GPIO_Pin {
public:
    GPIO_Pin();
    void init(PIN p);
    void set(void);
    void clear(void);

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

#endif
