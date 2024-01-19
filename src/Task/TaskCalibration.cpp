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
 * @brief  Calibration task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskCalibration.h"
#include <TTask.h>
#include <Board.h>
#include <StatisticValue.hpp>

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

/** Task to show calibration information on the serial console. */
static TTask gTask(taskFunc, nullptr, 20U, true);

/** Average calculation of the acceleration on x-axis. */
static StatisticValue<float, 5U> gAccXAvg(0.0F);

/** Average calculation of the acceleration on y-axis. */
static StatisticValue<float, 5U> gAccYAvg(0.0F);

/** Average calculation of the acceleration on z-axis. */
static StatisticValue<float, 5U> gAccZAvg(0.0F);

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

TaskBase* TaskCalibration::getTask()
{
    return &gTask;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Console output for calibration.
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
    unsigned long    timestamp       = millis();

    board.process();

    accDrv.getAcceleration(acceleration);
    gyroDrv.getGyro(gyro);

    gAccXAvg.update(acceleration.x);
    gAccYAvg.update(acceleration.y);
    gAccZAvg.update(acceleration.z);

    /* Using teleplot to visualization: https://github.com/nesnes/teleplot */
    Serial.print(">accX:");
    Serial.print(timestamp);
    Serial.print(":");
    Serial.print(gAccXAvg.getAvg(), 2);
    Serial.println(":m/s^2");

    Serial.print(">accY:");
    Serial.print(timestamp);
    Serial.print(":");
    Serial.print(gAccYAvg.getAvg(), 2);
    Serial.println(":m/s^2y");

    Serial.print(">accZ:");
    Serial.print(timestamp);
    Serial.print(":");
    Serial.print(gAccZAvg.getAvg(), 2);
    Serial.println(":m/s^2");
}
