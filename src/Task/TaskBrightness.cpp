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
 * @brief  Brightness control task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskBrightness.h"
#include <TTask.h>
#include <Board.h>
#include <Constants.h>

#include "TaskMotion.h"

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

static void taskFunc(void* par);
static void controlBrightness(Orientation orientation, Rotation rotation);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task handles the lamp brightness. */
static TTask gTask(taskFunc, nullptr, 20U, false);

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

TaskBase* TaskBrightness::getTask()
{
    return &gTask;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * The task function handles the lamp brightness.
 *
 * @param[in] par   Task parameter
 */
static void taskFunc(void* par)
{
    Orientation orientation = TaskMotion::getOrientation();
    Rotation    rotation    = TaskMotion::getRotation();

    /* Control brightness */
    controlBrightness(orientation, rotation);
}

/**
 * Control the pixel brightness by rotating the lamp around z-axis. The z-axis
 * must be horizontal to earth.
 *
 * @param[in] orientation   The lamp orientation state.
 * @param[in] rotation      The lamp rotation state.
 */
static void controlBrightness(Orientation orientation, Rotation rotation)
{
    if (ORIENTATION_HORIZONTAL == orientation)
    {
        Adafruit_NeoPixel& neoPixel   = Board::getInstance().getPixelDrv();
        uint8_t            brightness = neoPixel.getBrightness();
        bool               hasChanged = false;

        if (ROTATION_POSITIVE == rotation)
        {
            if (Constants::neoPixelMaxBrightness > brightness)
            {
                ++brightness;
                hasChanged = true;
            }
        }
        else if (ROTATION_NEGATIVE == rotation)
        {
            if (Constants::neoPixelMinBrightness < brightness)
            {
                --brightness;
                hasChanged = true;
            }
        }
        else
        {
            /* Nothing to do */
            ;
        }

        if (true == hasChanged)
        {
            neoPixel.setBrightness(brightness);
            neoPixel.show();
        }
    }
}
