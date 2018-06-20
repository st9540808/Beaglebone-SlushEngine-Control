/**
 * @file slushengine.cpp
 *
 */
/*
 * Based on https://github.com/Roboteurs/slushengine/tree/master/Slush
 */
/* Copyright (C) 2017 by Arjan van Vught mailto:info@raspberrypi-dmx.nl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <assert.h>
#if defined (__linux__)
#include <stdio.h>
#endif

extern "C" {
    #include "SPI.h"
}

#include "slushmotor.h"
#include "slushboard.h"

#include "l6470constants.h"
#include "gpio_pin.h"

extern GPIO_Pin motor_SPI_CS[7];

SlushMotor::SlushMotor(int nMotor, bool bUseSPI)
    : m_bIsBusy(false)
    , m_bIsConnected(false)
    // 6 for model D
    , mSpiChipSelect((assert(nMotor >= 0 && nMotor <= 6), motor_SPI_CS[nMotor]))
    , mUseL6480(nMotor >= 0 && nMotor <= 2 ? true : false)
{
    DEBUG_PRINT("Slush Motor %d initializing...", nMotor);

    m_nMotorNumber = nMotor;
    m_bUseSpiBusy = bUseSPI;

    switch (nMotor) {
    case 0:
        mPin = SLUSH_MTR0_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR0_CHIPSELECT);
        // m_nBusyPin = SLUSH_MTR0_BUSY;
        break;
    case 1:
        mPin = SLUSH_MTR1_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR1_CHIPSELECT);
        // m_nBusyPin = SLUSH_MTR1_BUSY;
        break;
    case 2:
        mPin = SLUSH_MTR2_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR2_CHIPSELECT);
        // m_nBusyPin = SLUSH_MTR2_BUSY;
        break;
    case 3:
        mPin = SLUSH_MTR3_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR3_CHIPSELECT);
        // m_nBusyPin = SLUSH_MTR3_BUSY;
        break;
    case 4:
        mPin = SLUSH_MTR4_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR4_CHIPSELECT);
        break;
    case 5:
        mPin = SLUSH_MTR5_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR5_CHIPSELECT);
        break;
    case 6:
        mPin = SLUSH_MTR6_CHIPSELECT;
        break;
    default:
        mPin = SLUSH_MTR0_CHIPSELECT;
        // mSpiChipSelect.init(SLUSH_MTR0_CHIPSELECT);
        // m_nBusyPin = SLUSH_MTR0_BUSY;
        break;
    }

    if (getParam(L6470_PARAM_CONFIG) == 0x2e88) {
        DEBUG_PRINT("Motor Drive Connected on GPIO %s", mPin.location);
        
        setOverCurrent(2000);
        setMicroSteps(16);
        setCurrent(70, 90, 100, 100);

        getStatus();
        this->free();

        m_bIsConnected = true;
    } else if (getParam(L6480_PARAM_CONFIG) == 0x2c88) {
        DEBUG_PRINT("High power motor Drive Connected on GPIO %s", mPin.location);

        setParam(L6480_PARAM_CONFIG, 0x3608);
        setCurrent(100, 120, 140, 140);
        setMicroSteps(16);

        DEBUG_PRINT("getStatus(): %#010x\n", getStatus());
        this->free();

        m_bIsConnected = true;
    } else {
        DEBUG_PRINT("received %ld", getParam(L6480_PARAM_CONFIG));
        FATAL("communication issues; check SPI configuration and cables");
    }
}

SlushMotor::~SlushMotor(void)
{
    free();
    m_bIsBusy = false;
    m_bIsConnected = false;
}

int SlushMotor::busyCheck(void)
{
    if (!m_bUseSpiBusy)
        FATAL("m_bUseSpiBusy should not be false, use SpiBusy");
    
    if (mUseL6480) {
        if (getParam(L6480_PARAM_STATUS) & L6470_STATUS_BUSY)
            return 0;
        else
            return 1;
    } else {
        if (getParam(L6470_PARAM_STATUS) & L6470_STATUS_BUSY)
            return 0;
        else
            return 1;
    }
}

uint8_t SlushMotor::SPIXfer(uint8_t data)
{
    unsigned char dataPacket[1];
    dataPacket[0] = (unsigned char) data;

    // bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);

    // bcm2835_gpio_clr(m_nSpiChipSelect);
    // bcm2835_spi_transfern(dataPacket, 1);
    // bcm2835_gpio_set(m_nSpiChipSelect);

    mSpiChipSelect.clear();
    dataPacket[0] = SPIDEV1_single_transfer(dataPacket[0]);
    mSpiChipSelect.set();
    
    DEBUG_PRINT("send: %u, recv: %u", data, dataPacket[0]);

    return (uint8_t) dataPacket[0];
}

/*
 * Roboteurs Slushengine Phyton compatible methods
 */

int SlushMotor::isBusy(void)
{
    return busyCheck();
}

void SlushMotor::setAsHome(void)
{
    resetPos();
}

void SlushMotor::setOverCurrent(unsigned int nCurrentmA)
{
    uint8_t OCValue = nCurrentmA / 375;

    if (OCValue > 0x0F) {
        OCValue = 0x0F;
    }

    setParam(L6470_PARAM_OCD_TH, OCValue);
}

void SlushMotor::softFree(void)
{
    softHiZ();
}

void SlushMotor::free(void)
{
    hardHiZ();
}

/*
 * Additional methods
 */
bool SlushMotor::IsConnected(void) const
{
    return m_bIsConnected;
}

bool SlushMotor::GetUseSpiBusy(void) const
{
    return m_bUseSpiBusy;
}

void SlushMotor::SetUseSpiBusy(bool bUseSpiBusy)
{
    m_bUseSpiBusy = bUseSpiBusy;
}
