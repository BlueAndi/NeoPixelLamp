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
 * @brief  Gyroscope driver
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HAL
 *
 * @{
 */

#ifndef GYROSCOPE_DRV_H
#define GYROSCOPE_DRV_H

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

/** The gyroscope driver provides the current gyroscope. */
class GyroscopeDrv
{
public:
    /**
     * Construct the driver instance.
     *
     * @param[in] sensor    The gyroscope sensor which to use.
     */
    GyroscopeDrv(Adafruit_Sensor& sensor) :
        m_sensor(sensor),
        m_isOffsetCompensationEnabled(false),
        m_offset({0.0F, 0.0F, 0.0F})
    {
    }

    /**
     * Destroy the driver instance.
     */
    ~GyroscopeDrv()
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
     * Get 3-axis gyroscope vector in degree/s.
     *
     * If the offset compensation is enabled, they will be considered for
     * all 3 axis.
     *
     * @param[out] gyro 3-axis gyroscope vector in degree/s.
     */
    void getGyro(sensors_vec_t& gyro);

private:
    Adafruit_Sensor& m_sensor;                      /**< Gyroscope sensor */
    bool             m_isOffsetCompensationEnabled; /**< Is offset compensation enabled? */
    sensors_vec_t    m_offset;                      /**< Offset for offset compensation. */

    /** Default constructor not allowed. */
    GyroscopeDrv();

    /** Copy constructor not allowed. */
    GyroscopeDrv(const GyroscopeDrv& other);

    /** Assingment operator not allowed. */
    GyroscopeDrv& operator=(const GyroscopeDrv& other);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* GYROSCOPE_DRV_H */
/** @} */
