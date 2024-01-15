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
 * @brief  Debug plot task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskDebugPlot.h"
#include <TTask.h>
#include <Board.h>

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

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task to show debug information on the serial console. */
static TTask gTask(taskFunc, nullptr, 100U, true);

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

TaskBase* TaskDebugPlot::getTask()
{
    return &gTask;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Console output for debugging purposes.
 *
 * @param[in] par   Task parameter
 */
static void taskFunc(void* par)
{
    Board&           board   = Board::getInstance();
    AccelerationDrv& accDrv  = board.getAccelerationSensor();
    GyroscopeDrv&    gyroDrv = board.getGyroscopeSensor();
    sensors_vec_t    acceleration;
    sensors_vec_t    gyro;
    float            absAcceleration = 0.0F;

    accDrv.getAcceleration(acceleration);
    gyroDrv.getGyro(gyro);

    absAcceleration = accDrv.getAbsAcceleration(&acceleration);

    Serial.print(acceleration.x, 2);
    Serial.print(",");
    Serial.print(acceleration.y, 2);
    Serial.print(",");
    Serial.println(acceleration.z, 2);
    Serial.print(",");
    Serial.println(absAcceleration, 2);

    Serial.print(gyro.x, 2);
    Serial.print(",");
    Serial.print(gyro.y, 2);
    Serial.print(",");
    Serial.println(gyro.z, 2);
}
