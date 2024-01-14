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
 * @brief  User interface task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskUI.h"
#include <TTask.h>
#include <Board.h>
#include <Constants.h>

#include "TaskModeHandler.h"

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
static void controlBrightness(float absAcc, float accZ, float gyroZ);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task handles the main user interface. */
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

TaskBase* TaskUI::getTask()
{
    return &gTask;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * The task function handles the main user interface.
 *
 * @param[in] par   Task parameter
 */
static void taskFunc(void* par)
{
    Board&           board           = Board::getInstance();
    AccelerationDrv& accDrv          = board.getAccelerationSensor();
    GyroscopeDrv&    gyroDrv         = board.getGyroscopeSensor();
    float            absAcceleration = 0.0F;
    float            absAccelDiff    = 0.0F;
    sensors_vec_t    accelerationVec;
    sensors_vec_t    gyroscopeVec;
    const float      MODE_CHANGE_THRESHOLD = 4.0F; /* Acceleration threshold in m/s^2, which triggers a mode change. */

    board.process();

    accDrv.getAcceleration(accelerationVec);
    gyroDrv.getGyro(gyroscopeVec);

    absAcceleration = accDrv.getAbsAcceleration(&accelerationVec);
    absAccelDiff    = fabs(absAcceleration - Constants::earthGravity);

    /* Control brightness */
    controlBrightness(absAcceleration, accelerationVec.z, gyroscopeVec.z);

    /* Does the user request a mode change by shaking the lamp? */
    if ((MODE_CHANGE_THRESHOLD <= absAccelDiff) &&            /* Lamp shall be shaked with a min. acceleration */
        (1.0F > fabs(absAcceleration - accelerationVec.z)) && /* Lamp shall be shaked along the z-axis */
        (20.0F > fabs(gyroscopeVec.x)) &&                     /* Lamp shall not be rotated at all */
        (20.0F > fabs(gyroscopeVec.y)) && (20.0F > fabs(gyroscopeVec.z)))
    {
        TaskModeHandler::selectNextMode();
    }
}

/**
 * Control the NeoPixel ring LEDs brightness by rotating the lamp
 * around z-axis. The z-axis must be horizontal to earth.S
 *
 * @param[in] absAcc    Absolute acceleration value in m/s^2
 * @param[in] accZ      Acceleration along the z-axis in m/s^2
 * @param[in] gyroZ     Angle speed around the z-axis in degree/s
 */
static void controlBrightness(float absAcc, float accZ, float gyroZ)
{
    Adafruit_NeoPixel& neoPixel             = Board::getInstance().getPixelDrv();
    const float        MIN_ANGULAR_VELOCITY = 20.0F;
    const float        MIN_ACCELERATION     = 1.6F;
    const float        MIN_ABS_ACCELERATION = Constants::earthGravity - 1.6F;
    uint8_t            brightness           = 0;
    bool               hasChanged           = false;

    /* Nearly no acceleration shall take place. */
    if (MIN_ABS_ACCELERATION < fabs(absAcc - Constants::earthGravity))
    {
        /* Do nothing */
    }
    /* The lamp shall be horizontal to earth. That means the z-axis
     * acceleration should be low, otherwise it would be around 9.81 m/s^2.
     */
    else if (MIN_ACCELERATION > fabs(accZ))
    {
        /* If the lamp is turned right, the brightness increases. */
        if (MIN_ANGULAR_VELOCITY < gyroZ)
        {
            brightness = neoPixel.getBrightness();

            if (Constants::neoPixelMaxBrightness > brightness)
            {
                ++brightness;

                hasChanged = true;
            }
        }
        /* If the lamp is turned left, the brightness decreases. */
        else if (-MIN_ANGULAR_VELOCITY > gyroZ)
        {
            brightness = neoPixel.getBrightness();

            if (Constants::neoPixelMinBrightness < brightness)
            {
                --brightness;

                hasChanged = true;
            }
        }
        /* Nothing to do */
        else
        {
            ;
        }
    }

    if (true == hasChanged)
    {
        Serial.print("Change brightness to ");
        Serial.println(brightness);

        neoPixel.setBrightness(brightness);
        neoPixel.show();
    }
}
