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
 * @brief  Settings realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef SETTINGS_H
#define SETTINGS_H

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

/** This class handles settings stored in the EEPROM. */
class Settings
{
public:
    /**
     * Constructs the settings adapter.
     */
    Settings()
    {
    }

    /**
     * Destroys the button A adapter.
     */
    ~Settings()
    {
    }

    /**
     * Initialize the settings.
     *
     * If the settings are invalid or not compatible to the settings, the
     * default values will be written!
     */
    void init();

    /**
     * Get x-axis acceleration offset.
     *
     * @return x-axis acceleration offset in m/s^2
     */
    float getAccelerationXOffset() const;

    /**
     * Set x-axis acceleration offset.
     *
     * @param[in] offset    x-axis acceleration offset in m/s^2
     */
    void setAccelerationXOffset(float offset) const;

    /**
     * Get y-axis acceleration offset.
     *
     * @return y-axis acceleration offset in m/s^2
     */
    float getAccelerationYOffset() const;

    /**
     * Set y-axis acceleration offset.
     *
     * @param[in] offset    y-axis acceleration offset in m/s^2
     */
    void setAccelerationYOffset(float offset) const;

    /**
     * Get z-axis acceleration offset.
     *
     * @return z-axis acceleration offset in m/s^2
     */
    float getAccelerationZOffset() const;

    /**
     * Set z-axis acceleration offset.
     *
     * @param[in] offset    z-axis acceleration offset in m/s^2
     */
    void setAccelerationZOffset(float offset) const;

private:

    /**
     * Get magic pattern.
     *
     * @return Magic pattern
     */
    uint32_t getMagicPattern() const;

    /**
     * Set magic pattern.
     *
     * @param[in] value Magic pattern
     */
    void setMagicPattern(uint32_t value) const;

    /**
     * Get data version.
     *
     * @return Data version
     */
    uint8_t getDataVersion() const;

    /**
     * Set data version.
     *
     * @param[in] value Data version
     */
    void setDataVersion(uint8_t value) const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SETTINGS_H */
/** @} */
