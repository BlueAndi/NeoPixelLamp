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
 * @brief  Debug task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskDebug.h"
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

TaskBase* TaskDebug::getTask()
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
    MagnetometerDrv& magDrv  = board.getMagnetometerSensor();
    GyroscopeDrv&    gyroDrv = board.getGyroscopeSensor();
    TemperatureDrv&  tempDrv = board.getTemperatureSensor();
    sensors_vec_t    acceleration;
    sensors_vec_t    magnetic;
    sensors_vec_t    gyro;
    float            absAcceleration = 0.0F;
    float            temperature     = tempDrv.getTemperature();
    float            heading         = magDrv.getHeading();

    accDrv.getAcceleration(acceleration);
    magDrv.getMagnetic(magnetic);
    gyroDrv.getGyro(gyro);

    absAcceleration = accDrv.getAbsAcceleration(&acceleration);

    Serial.print("Acceleration values  : X = ");
    Serial.print(acceleration.x, 2);
    Serial.print(" Y = ");
    Serial.print(acceleration.y, 2);
    Serial.print(" Z = ");
    Serial.println(acceleration.z, 2);

    Serial.print("Magnetic values      : X = ");
    Serial.print(magnetic.x, 2);
    Serial.print(" Y = ");
    Serial.print(magnetic.y, 2);
    Serial.print(" Z = ");
    Serial.println(magnetic.z, 2);

    Serial.print("Gyroscope values     : X = ");
    Serial.print(gyro.x, 2);
    Serial.print(" Y = ");
    Serial.print(gyro.y, 2);
    Serial.print(" Z = ");
    Serial.println(gyro.z, 2);

    Serial.print("Absolute acceleration: ");
    Serial.print(absAcceleration, 2);
    Serial.println(" m/s^2");

    Serial.print("Temperature          : ");
    Serial.print(temperature, 1);
    Serial.println(" degree celsius");

    Serial.print("Heading              : ");
    Serial.print(heading, 2);
    Serial.println(" degree");

    Serial.println("");
}
