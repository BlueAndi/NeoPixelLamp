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
 * @brief  Magnetometer driver
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HAL
 *
 * @{
 */

#ifndef MAGNETOMETER_DRV_H
#define MAGNETOMETER_DRV_H

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

/** The magnetometer driver provides the current magnetometer. */
class MagnetometerDrv
{
public:
    /**
     * Construct the driver instance.
     *
     * @param[in] sensor    The magnetometer sensor which to use.
     */
    MagnetometerDrv(Adafruit_Sensor& sensor) :
        m_sensor(sensor),
        m_isOffsetCompensationEnabled(false),
        m_offset({0.0F, 0.0F, 0.0F})
    {
    }

    /**
     * Destroy the driver instance.
     */
    ~MagnetometerDrv()
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
     * Get 3-axis magnetic vector in gauss.
     * 
     * If the offset compensation is enabled, they will be considered for
     * all 3 axis.
     *
     * @param[out] magnetic 3-axis magnetic vector in gauss.
     */
    void getMagnetic(sensors_vec_t& magnetic);

    /**
     * Get your heading, using Earth's magnetic field.
     * It only works if the sensor is flat (z-axis normal to Earth).
     * Additionally, you may need to add or subtract a declination
     * angle to get the heading normalized to your location.
     * See: http://www.ngdc.noaa.gov/geomag/declination.shtml
     * 
     * If the offset compensation is enabled, they will be considered for
     * all 3 axis.
     *
     * @return Heading in degree.
     */
    float getHeading();

private:
    Adafruit_Sensor& m_sensor;                      /**< Magnetometerr sensor */
    bool             m_isOffsetCompensationEnabled; /**< Is offset compensation enabled? */
    sensors_vec_t    m_offset;                      /**< Offset for offset compensation. */

    /** Default constructor not allowed. */
    MagnetometerDrv();

    /** Copy constructor not allowed. */
    MagnetometerDrv(const MagnetometerDrv& other);

    /** Assignment operator not allowed. */
    MagnetometerDrv& operator=(const MagnetometerDrv& other);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MAGNETOMETER_DRV_H */
/** @} */
