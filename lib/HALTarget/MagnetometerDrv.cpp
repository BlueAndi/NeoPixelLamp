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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "MagnetometerDrv.h"

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

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void MagnetometerDrv::getMagnetic(sensors_vec_t& magnetic)
{
    sensors_event_t magneticEvent;

    while (false == m_sensor.getEvent(&magneticEvent))
    {
        ;
    }

    magnetic = magneticEvent.magnetic;

    if (true == m_isOffsetCompensationEnabled)
    {
        magnetic.x += m_offset.x;
        magnetic.y += m_offset.y;
        magnetic.z += m_offset.z;
    }
}

float MagnetometerDrv::getHeading()
{
    sensors_vec_t magnetic;
    float         heading = 0.0F;

    getMagnetic(magnetic);

    if (0.0f < magnetic.y)
    {
        heading = 90.0F - (atan(magnetic.x / magnetic.y) * (180.0F / PI));
    }
    else if (0.0f > magnetic.y)
    {
        heading = -1.0F * (atan(magnetic.x / magnetic.y) * (180.0F / PI));
    }
    else /* magnetic.y = 0.0f */
    {
        if (0.0f > magnetic.x)
        {
            heading = 180.0F;
        }
        else
        {
            heading = 0.0F;
        }
    }

    return heading;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/