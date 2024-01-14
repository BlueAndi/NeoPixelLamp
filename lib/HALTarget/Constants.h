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
 * @brief  General constants
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HAL
 *
 * @{
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/

/**
 * General constant values.
 */
namespace Constants
{
    /** I2C address of the LSM9DS0 magnetic sensor. */
    constexpr int32_t lsm9ds0I2cAddr = 0xD4U;

    /** Number of NeoPixel's on the strip. */
    constexpr uint16_t neoPixelStripLength = 16U;

    /** Max. brightness of the pixels. */
    constexpr uint8_t neoPixelMaxBrightness = 160U;

    /** Min. brightness of the pixels. */
    constexpr uint8_t neoPixelMinBrightness = 10U;

    /** Default brightness of the pixels. */
    constexpr uint8_t neoPixelDefaultBrightness = 120U;

    /** Earth gravity acceleration in m/s^2 */
    constexpr float earthGravity = 9.81F;

    /** Max. temperature in degree celsius of the temperature sensor. */
    constexpr float temperatureMax = 85.0F;

    /** Min. temperature in degree celsius of the temperature sensor */
    constexpr float temperatureMin = -40.0F;

}; /* namespace Constants */

#endif /* CONSTANTS_H */
/** @} */
