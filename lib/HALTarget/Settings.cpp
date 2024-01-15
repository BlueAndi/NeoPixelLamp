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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Settings.h"
#include <avr/eeprom.h>

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

template<typename T>
static T readData(const void* addr);
template<typename T>
static void writeData(const void* addr, T data);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/**
 * Magic pattern address in EEPROM.
 */
static const void* MAGIC_PATTERN_ADDR = static_cast<const void*>(0x0000);

/**
 * Magic pattern size in byte.
 */
static const size_t MAGIC_PATTERN_SIZE = sizeof(uint32_t);

/**
 * Data version address in EEPROM.
 */
static const void* DATA_VERSION_ADDR = &static_cast<const uint8_t*>(MAGIC_PATTERN_ADDR)[MAGIC_PATTERN_SIZE];

/**
 * Data version size in byte.
 */
static const size_t DATA_VERSION_SIZE = sizeof(uint8_t);

/**
 * x-axis acceleration offset address.
 */
static const void* ACC_X_OFFSET_ADDR = &static_cast<const uint8_t*>(DATA_VERSION_ADDR)[DATA_VERSION_SIZE];

/**
 * x-axis acceleration offset size in byte.
 */
static const size_t ACC_X_OFFSET_SIZE = sizeof(float);

/**
 * y-axis acceleration offset address.
 */
static const void* ACC_Y_OFFSET_ADDR = &static_cast<const uint8_t*>(ACC_X_OFFSET_ADDR)[ACC_X_OFFSET_SIZE];

/**
 * y-axis acceleration offset size in byte.
 */
static const size_t ACC_Y_OFFSET_SIZE = sizeof(float);

/**
 * z-axis acceleration offset address.
 */
static const void* ACC_Z_OFFSET_ADDR = &static_cast<const uint8_t*>(ACC_Y_OFFSET_ADDR)[ACC_Y_OFFSET_SIZE];

/**
 * z-axis acceleration offset size in byte.
 */
static const size_t ACC_Z_OFFSET_SIZE = sizeof(float);

/* ---------- */

/**
 * Default acceleration offset in m/s^2, used for initialization.
 */
static const float DEFAULT_ACCELERATION_OFFSET = 0.0F;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void Settings::init()
{
    uint32_t magicPattern = readData<uint32_t>(MAGIC_PATTERN_ADDR);
    uint8_t  dataVersion  = readData<uint8_t>(DATA_VERSION_ADDR);

    if ((MAGIC_PATTERN != magicPattern) || (DATA_VERSION != dataVersion))
    {
        /* Write default values. */
        setAccelerationXOffset(DEFAULT_ACCELERATION_OFFSET);
        setAccelerationYOffset(DEFAULT_ACCELERATION_OFFSET);
        setAccelerationZOffset(DEFAULT_ACCELERATION_OFFSET);

        /* Mark data in EEPROM as valid. */
        writeData<uint32_t>(MAGIC_PATTERN_ADDR, MAGIC_PATTERN);
        writeData<uint8_t>(DATA_VERSION_ADDR, DATA_VERSION_SIZE);
    }
}

float Settings::getAccelerationXOffset() const
{
    return readData<float>(ACC_X_OFFSET_ADDR);
}

void Settings::setAccelerationXOffset(float offset) const
{
    writeData<float>(ACC_X_OFFSET_ADDR, offset);
}

float Settings::getAccelerationYOffset() const
{
    return readData<float>(ACC_Y_OFFSET_ADDR);
}

void Settings::setAccelerationYOffset(float offset) const
{
    writeData<float>(ACC_Y_OFFSET_ADDR, offset);
}

float Settings::getAccelerationZOffset() const
{
    return readData<float>(ACC_X_OFFSET_ADDR);
}

void Settings::setAccelerationZOffset(float offset) const
{
    writeData<float>(ACC_Z_OFFSET_ADDR, offset);
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

/**
 * Read data from EEPROM.
 *
 * @tparam T    Data type
 *
 * @param[in]   addr    Address in the EEPROM.
 *
 * @return Data
 */
template<typename T>
static T readData(const void* addr)
{
    T value;

    eeprom_read_block(&value, addr, sizeof(T));

    return value;
}

/**
 * Read data from EEPROM.
 *
 * @tparam T    Data type
 *
 * @param[in]   addr    Address in the EEPROM.
 * @param[in]   data    Data to write.
 */
template<typename T>
static void writeData(const void* addr, T data)
{
    eeprom_write_block(addr, &data, sizeof(T));
}
