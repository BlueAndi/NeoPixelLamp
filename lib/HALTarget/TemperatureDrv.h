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
 * @brief  Temperature driver
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HAL
 *
 * @{
 */

#ifndef TEMPERATURE_DRV_H
#define TEMPERATURE_DRV_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Adafruit_Sensor.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The temperature driver provides the current temperature. */
class TemperatureDrv
{
public:

    /**
     * Construct the driver instance.
     * 
     * @param[in] sensor    The temperature sensor which to use.
     */
    TemperatureDrv(Adafruit_Sensor& sensor) : m_sensor(sensor)
    {
    }

    /**
     * Destroy the driver instance.
     */
    ~TemperatureDrv()
    {
    }

    /**
     * Get temperature in °C.
     *
     * @return Temperature in °C
     */
    float getTemperature();

private:
    Adafruit_Sensor& m_sensor; /**< Temperature sensor */

    /** Default constructor not allowed. */
    TemperatureDrv();

    /** Copy constructor not allowed. */
    TemperatureDrv(const TemperatureDrv& other);

    /** Assignment operator not allowed. */
    TemperatureDrv& operator=(const TemperatureDrv& other);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TEMPERATURE_DRV_H */
/** @} */
