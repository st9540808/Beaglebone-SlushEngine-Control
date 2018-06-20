#include "gpio_pin.h"

volatile uint8_t* GPIO_Pin::CM_PER_base = nullptr;
volatile uint8_t* GPIO_Pin::GPIO2_base = nullptr;
volatile uint8_t* GPIO_Pin::GPIO1_base = nullptr;

GPIO_Pin motor_SPI_CS[7];
GPIO_Pin MTR_RESET;

GPIO_Pin::GPIO_Pin()
    : MAP_SIZE(sysconf(_SC_PAGESIZE))
    , MAP_MASK(MAP_SIZE - 1)
{}

void GPIO_Pin::init(PIN p)
{
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
            DEBUG_PRINT("%#010x", *(uint32_t *)(CM_PER_base + CM_PER_GPIO2_CLKCTRL));
        
        if (!GPIO2_base) {
            GPIO2_base = (uint8_t *) mmap(0, 4096, PROT_READ | PROT_WRITE,
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
            GPIO1_base = (uint8_t *) mmap(0, 4096, PROT_READ | PROT_WRITE,
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

void GPIO_Pin::set(void)
{
    *(uint32_t *)(SETDATAOUT_addr) = 1 << this->bank;
}

void GPIO_Pin::clear(void)
{
    *(uint32_t *)(CLEARDATAOUT_addr) = 1 << this->bank;
}
