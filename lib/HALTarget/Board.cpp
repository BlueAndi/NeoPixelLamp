/* MIT License
 *
 * Copyright (c) 2016 - 2024 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Hardware abstraction layer for the target MCU.
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Board.h>
#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "Constants.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Pin number of all used pins. */
namespace Pin
{
    /** NeoPixel output pin. */
    constexpr uint8_t neoPixelOutPinNo = 6U;

    /** Onboard LED pin. */
    constexpr uint8_t onboardLedPinNo = 7U;

}; /* Namespace Pin */

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool Board::init()
{
    bool    isSuccessful = true;
    uint8_t retry        = 3U;

    pinMode(Pin::onboardLedPinNo, OUTPUT);

    /* Enable the onboard LED till setup. */
    ledOn();

    /* Disable some peripherals to reduce power consumption */
    power_adc_disable();
    power_spi_disable();
    power_timer2_disable();
    power_timer3_disable();

    /* Initialize all pixels and disable them. */
    m_neoPixel.begin();
    m_neoPixel.show();
    m_neoPixel.setBrightness(Constants::neoPixelDefaultBrightness);

    /* Make sure the magnetic sensor is found. */
    while ((0 < retry) && (false == m_magneticSensorDrv.begin()))
    {
        --retry;
    }

    /* Magnetic sensor not found? */
    if (0 == retry)
    {
        isSuccessful = false;
    }
    else
    {
        /* Set accelerometer anti-alias filter bandwidth to 50 Hz */
#if 0
        uint8_t reg = lsm.read8(XMTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG2_XM);
        reg &= 0x3f;
        reg |= 0xc0;
        lsm.write8(XMTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG2_XM, reg);
#endif

        /* 1.) Set the accelerometer range. */
        m_magneticSensorDrv.setupAccel(Adafruit_LSM9DS0::LSM9DS0_ACCELRANGE_2G);

        /* 2.) Set the magnetometer sensitivity. */
        m_magneticSensorDrv.setupMag(Adafruit_LSM9DS0::LSM9DS0_MAGGAIN_2GAUSS);

        /* 3.) Setup the gyroscope. */
        m_magneticSensorDrv.setupGyro(Adafruit_LSM9DS0::LSM9DS0_GYROSCALE_245DPS);
    }

    /* Disable the onboard LED after setup. */
    ledOff();

    return isSuccessful;
}

void Board::ledOn()
{
    digitalWrite(Pin::onboardLedPinNo, HIGH);
}

void Board::ledOff()
{
    digitalWrite(Pin::onboardLedPinNo, LOW);
}

void Board::reset()
{
    /* Enable watchdog and wait for reset. */
    wdt_enable(WDTO_15MS);
    while (1)
        ;
}

void Board::process()
{
    /* Read all sensors data. */
    m_magneticSensorDrv.read();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

Board::Board() :
    m_magneticSensorDrv(Constants::lsm9ds0I2cAddr),
    m_neoPixel(Constants::neoPixelStripLength, Pin::neoPixelOutPinNo, NEO_GRB + NEO_KHZ800),
    m_accDrv(m_magneticSensorDrv.getAccel()),
    m_gyroDrv(m_magneticSensorDrv.getGyro()),
    m_magDrv(m_magneticSensorDrv.getMag()),
    m_tmpDrv(m_magneticSensorDrv.getTemp())
{
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
