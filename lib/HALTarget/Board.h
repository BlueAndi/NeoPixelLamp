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
 *
 * @addtogroup HAL
 *
 * @{
 */
#ifndef BOARD_H
#define BOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM9DS0.h>

#include "AccelerationDrv.h"
#include "GyroscopeDrv.h"
#include "MagnetometerDrv.h"
#include "TemperatureDrv.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The concrete physical board.
 */
class Board
{
public:
    /**
     * Get board instance.
     *
     * @return Board instance
     */
    static Board& getInstance()
    {
        static Board instance; /* idiom */

        return instance;
    }

    /**
     * Initialize the hardware.
     *
     * @return If successful it will return true otherwise false.
     */
    bool init();

    /**
     * Enable the onboard LED.
     */
    void ledOn();

    /**
     * Disable the onboard LED.
     */
    void ledOff();

    /**
     * Perform a reset.
     */
    static void reset();

    /**
     * Process all sensor drivers.
     * This shall be called periodically.
     */
    void process();

    /**
     * Get the acceleration sensor.
     *
     * @return Acceleration sensor
     */
    AccelerationDrv& getAccelerationSensor()
    {
        return m_accDrv;
    }

    /**
     * Get the gyroscope sensor.
     *
     * @return Gyroscope sensor
     */
    GyroscopeDrv& getGyroscopeSensor()
    {
        return m_gyroDrv;
    }

    /**
     * Get the magnetometer sensor.
     *
     * @return Magnetometer sensor
     */
    MagnetometerDrv& getMagnetometerSensor()
    {
        return m_magDrv;
    }

    /**
     * Get the temperature sensor.
     *
     * @return Temperature sensor
     */
    TemperatureDrv& getTemperatureSensor()
    {
        return m_tmpDrv;
    }

    /**
     * Get the NeoPixel strip driver.
     *
     * @return NeoPixel strip driver
     */
    Adafruit_NeoPixel& getPixelDrv()
    {
        return m_neoPixel;
    }

private:
    Adafruit_LSM9DS0  m_magneticSensorDrv; /**< LSM9DS0 magnetic sensor driver, I2C address 0xD4. */
    Adafruit_NeoPixel m_neoPixel;          /**< NeoPixel strip driver. */
    AccelerationDrv   m_accDrv;            /**< Acceleration driver which uses the LSM9DS0. */
    GyroscopeDrv      m_gyroDrv;           /**< Gyroscope driver which uses the LSM9DS0. */
    MagnetometerDrv   m_magDrv;            /**< Magnetometer driver which uses the LSM9DS0. */
    TemperatureDrv    m_tmpDrv;            /**< Temperature driver which uses the LSM9DS0. */

    /**
     * Constructs the concrete board.
     */
    Board();

    /**
     * Destroys the concrete board.
     */
    ~Board()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BOARD_H */
/** @} */
