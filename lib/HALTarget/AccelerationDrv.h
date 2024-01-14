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
 * @brief  Acceleration driver
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HAL
 *
 * @{
 */

#ifndef ACCELERATION_DRV_H
#define ACCELERATION_DRV_H

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

/** The acceleration driver provides the current acceleration. */
class AccelerationDrv
{
public:
    /**
     * Construct the driver instance.
     *
     * @param[in] sensor    The acceleration sensor which to use.
     */
    AccelerationDrv(Adafruit_Sensor& sensor) :
        m_sensor(sensor),
        m_isOffsetCompensationEnabled(false),
        m_offset({0.0F, 0.0F, 0.0F})
    {
    }

    /**
     * Destroy the driver instance.
     */
    ~AccelerationDrv()
    {
    }

    /**
     * Enable the offset compensation.
     */
    void enableOffsetCompensation()
    {
        m_isOffsetCompensationEnabled = true;
    }

    /**
     * Disable the offset compensation;
     */
    void disableOffsetCompensation()
    {
        m_isOffsetCompensationEnabled = false;
    }

    /**
     * Set offset for compensation.
     * 
     * @param[in] offset    Offset vector for compensation.
     */
    void setOffset(const sensors_vec_t& offset)
    {
        m_offset = offset;
    }

    /**
     * Get 3-axis acceleration vector in m/s^2.
     * 
     * If the offset compensation is enabled, they will be considered for
     * all 3 axis.
     *
     * @param[out]  3-axis acceleration vector in m/s^2
     */
    void getAcceleration(sensors_vec_t& acceleration);

    /**
     * Get absolute acceleration in m/s^2.
     * If the acceleration vector is given as parameter, it will return its
     * absolute value. Otherwise it will automatically retrieve the current
     * acceleration vector and return its absolute value.
     * 
     * If the offset compensation is enabled, they will be considered for
     * all 3 axis.
     * 
     * @param[in] acceleration  3-axis acceleration vector.
     *
     * @return Absolute acceleration value in m/s^2.
     */
    float getAbsAcceleration(sensors_vec_t* acceleration = nullptr);

private:
    Adafruit_Sensor& m_sensor;                      /**< Acceleration sensor */
    bool             m_isOffsetCompensationEnabled; /**< Is offset compensation enabled? */
    sensors_vec_t    m_offset;                      /**< Offset for offset compensation. */

    AccelerationDrv();
    AccelerationDrv(const AccelerationDrv& other);
    AccelerationDrv& operator=(const AccelerationDrv& other);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ACCELERATION_DRV_H */
/** @} */
