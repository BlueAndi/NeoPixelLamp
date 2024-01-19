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
 * @brief  Motion task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskMotion.h"
#include <TTask.h>
#include <Board.h>
#include <Constants.h>

#include "TaskModeHandler.h"
#include "StatisticValue.hpp"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/**
 * This type defines the different shake directions.
 */
typedef enum
{
    SHAKE_DIRECTION_NOTHING = 0, /**< No lamp movement. */
    SHAKE_DIRECTION_TO_BOTTOM,   /**< Lamp moves to earth ground. */
    SHAKE_DIRECTION_TO_HEAD      /**< Lamp moves away from earth ground. */

} ShakeDirection;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void        taskFunc(void* par);
static Orientation calculateOrientation(const sensors_vec_t& accVec);
static Rotation    calculateRotation(float gyroValue);
static bool        isShakeDetected(float accZ, const sensors_vec_t& gyroVec);

#ifdef DEBUG
static void printInfo();
#endif /* DEBUG */

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task handles the main user interface. */
static TTask gTask(taskFunc, nullptr, 20U, false);

/** Average calculation of the acceleration on x-axis. */
static StatisticValue<float, 5U> gAccXAvg(0.0F);

/** Average calculation of the acceleration on y-axis. */
static StatisticValue<float, 5U> gAccYAvg(0.0F);

/** Average calculation of the acceleration on z-axis. */
static StatisticValue<float, 5U> gAccZAvg(0.0F);

/** Average calculation of the gyro on x-axis. */
static StatisticValue<float, 2U> gGyroXAvg(0.0F);

/** Average calculation of the gyro on y-axis. */
static StatisticValue<float, 2U> gGyroYAvg(0.0F);

/** Average calculation of the gyro on z-axis. */
static StatisticValue<float, 2U> gGyroZAvg(0.0F);

/** Current lamp orientation state. */
static Orientation gOrientationState = ORIENTATION_UNKNOWN;

/** Current lamp rotation state. */
static Rotation gRotationState = ROTATION_NO;

/** Flag indicates that the user shaked the lamp. */
static bool gIsShakeDetected = false;

/** Current lamp shake direction. */
static ShakeDirection gShakeDirection = SHAKE_DIRECTION_NOTHING;

/** Timestamp of shake direction start to earth ground in ms. */
static unsigned long gShakeTimestamp = 0U;

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

TaskBase* TaskMotion::getTask()
{
    return &gTask;
}

Orientation TaskMotion::getOrientation()
{
    return gOrientationState;
}

Rotation TaskMotion::getRotation()
{
    return gRotationState;
}

bool TaskMotion::isShakeDetected()
{
    bool result = gIsShakeDetected;

    gIsShakeDetected = false;

    return result;
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
    Board&           board   = Board::getInstance();
    AccelerationDrv& accDrv  = board.getAccelerationSensor();
    GyroscopeDrv&    gyroDrv = board.getGyroscopeSensor();
    sensors_vec_t    accVec;
    sensors_vec_t    gyroVec;
    sensors_vec_t    accAvgVec;
    sensors_vec_t    gyroAvgVec;

    board.process();

    accDrv.getAcceleration(accVec);
    gyroDrv.getGyro(gyroVec);

    /* Calculate the average of the acceleration and gyro values. */
    gAccXAvg.update(accVec.x);
    gAccYAvg.update(accVec.y);
    gAccZAvg.update(accVec.z);

    gGyroXAvg.update(gyroVec.x);
    gGyroYAvg.update(gyroVec.y);
    gGyroZAvg.update(gyroVec.z);

    accAvgVec.x = gAccXAvg.getAvg();
    accAvgVec.y = gAccYAvg.getAvg();
    accAvgVec.z = gAccZAvg.getAvg();

    gyroAvgVec.x = gAccXAvg.getAvg();
    gyroAvgVec.y = gAccYAvg.getAvg();
    gyroAvgVec.z = gAccZAvg.getAvg();

    /* Calculate the lamp orientation. */
    gOrientationState = calculateOrientation(accAvgVec);

    /* Calculate the lamp rotation. */
    gRotationState = calculateRotation(gGyroZAvg.getAvg());

    /* Detect lamp shake. */
    if (true == isShakeDetected(accAvgVec.z, gyroAvgVec))
    {
        gIsShakeDetected = true;
    }

#ifdef DEBUG
    printInfo();
#endif /* DEBUG */
}

/**
 * Calculate the lamp orientation.
 *
 * @param[in] accVec    Acceleration vector of 3-axis in [m/s^2]
 *
 * @return Lamp orientation
 */
static Orientation calculateOrientation(const sensors_vec_t& accVec)
{
    float       accToEarthByXY           = sqrtf(accVec.x * accVec.x + accVec.y * accVec.y); /* [m/s^2] */
    const float ACC_VERTICAL_THRESHOLD   = Constants::earthGravity / 2.0F;                   /* [m/s^2] */
    const float ACC_HORIZONTAL_THRESHOLD = Constants::earthGravity / 2.0F;                   /* [m/s^2] */
    Orientation result                   = ORIENTATION_UNKNOWN;

    /* If the lamp is vertical on bottom ...
     * The acceleration on the z-axis should be around positive earth gravity acceleration.
     * The acceleration on the x- and y-axis relative to earth should be around 0. Always positive value!
     */
    if ((ACC_VERTICAL_THRESHOLD < accVec.z) && (ACC_HORIZONTAL_THRESHOLD > accToEarthByXY))
    {
        result = ORIENTATION_VERTICAL_ON_BOTTOM;
    }
    /* If the lamp is vertical on head ...
     * The acceleration on the z-axis should be around negative earth gravity acceleration.
     * The acceleration on the x- and y-axis relative to earth should be around 0. Always positive value!
     */
    else if ((-ACC_VERTICAL_THRESHOLD > accVec.z) && (ACC_HORIZONTAL_THRESHOLD > accToEarthByXY))
    {
        result = ORIENTATION_VERTICAL_ON_HEAD;
    }
    /* If the lamp is horizontal ...
     * The acceleration on the z-axis should be around 0.
     * The acceleration on the x- and y-axis relative to earth should be around earth gravity acceleration. Always
     * positive value!
     */
    else if ((ACC_HORIZONTAL_THRESHOLD > accVec.z) && (-ACC_HORIZONTAL_THRESHOLD < accVec.z) &&
             (ACC_VERTICAL_THRESHOLD < accToEarthByXY))
    {
        result = ORIENTATION_HORIZONTAL;
    }
    /* Any other orientation will be "unknown". */
    else
    {
        result = ORIENTATION_UNKNOWN;
    }

    return result;
}

/**
 * Calculate the lamp rotation.
 *
 * @param[in] gyroValue The value of the gyro axis which to use.
 *
 * @return Lamp rotation
 */
static Rotation calculateRotation(float gyroValue)
{
    const float GYRO_THRESHOLD = 4.0F; /* [°/s] */
    Rotation    result         = ROTATION_NO;

    if (GYRO_THRESHOLD < gyroValue)
    {
        result = ROTATION_POSITIVE;
    }
    else if (-GYRO_THRESHOLD > gyroValue)
    {
        result = ROTATION_NEGATIVE;
    }
    else
    {
        result = ROTATION_NO;
    }

    return result;
}

static bool isShakeDetected(float accZ, const sensors_vec_t& gyroVec)
{
    bool                result            = false;
    float               accZWithoutEarthG = accZ - Constants::earthGravity; /* [m/s^2] */
    unsigned long       timestamp         = millis();                       /* [ms] */
    const unsigned long TIME_WINDOW       = 500U;                           /* [ms] */
    const float         MODE_CHANGE_ACC_THRESHOLD =
        Constants::earthGravity / 2.0F;     /* Acceleration threshold [m/s^2], which triggers a mode change. */
    const float MAX_GYRO_THRESHOLD = 20.0F; /* Max. allowed angle velocity [°/s] for mode change. */

    /* In the first step, the user must shake the lamp on the z-axis to
     * earth direction and in the second step in the other direction.
     * This must be within a defined time window.
     */
    if (TIME_WINDOW < (timestamp - gShakeTimestamp))
    {
        gShakeDirection = SHAKE_DIRECTION_NOTHING;
        gShakeTimestamp = timestamp;
    }
    else if (0U == gShakeDirection)
    {
        if ((-MODE_CHANGE_ACC_THRESHOLD > accZWithoutEarthG) && /* Lamp shall be shaked along the z-axis. */
            (MAX_GYRO_THRESHOLD >= fabsf(gyroVec.x)) &&         /* Lamp shall not be rotated around x-axis. */
            (MAX_GYRO_THRESHOLD >= fabsf(gyroVec.y)) &&         /* Lamp shall not be rotated around y-axis. */
            (MAX_GYRO_THRESHOLD > fabsf(gyroVec.z)))            /* Lamp shall not be rotated around z-axis. */
        {
            gShakeDirection = SHAKE_DIRECTION_TO_BOTTOM;
        }
    }
    else if (1U == gShakeDirection)
    {
        if ((MODE_CHANGE_ACC_THRESHOLD < accZWithoutEarthG) && /* Lamp shall be shaked along the z-axis. */
            (MAX_GYRO_THRESHOLD >= fabsf(gyroVec.x)) &&        /* Lamp shall not be rotated around x-axis. */
            (MAX_GYRO_THRESHOLD >= fabsf(gyroVec.y)) &&        /* Lamp shall not be rotated around y-axis. */
            (MAX_GYRO_THRESHOLD > fabsf(gyroVec.z)))           /* Lamp shall not be rotated around z-axis. */
        {
            gShakeDirection = SHAKE_DIRECTION_TO_HEAD;
        }
    }
    else
    {
        result = true;

        gShakeDirection = SHAKE_DIRECTION_NOTHING;
        gShakeTimestamp = timestamp;
    }

    return result;
}

#ifdef DEBUG

/**
 * Print debug information to the console.
 */
static void printInfo()
{
    static Orientation oldOrientation = ORIENTATION_UNKNOWN;
    static Rotation    oldRotation    = ROTATION_NO;

    if (oldOrientation != gOrientationState)
    {
        switch (gOrientationState)
        {
        case ORIENTATION_UNKNOWN:
            Serial.println(F("ORIENTATION_UNKNOWN"));
            break;

        case ORIENTATION_VERTICAL_ON_BOTTOM:
            Serial.println(F("ORIENTATION_VERTICAL_ON_BOTTOM"));
            break;

        case ORIENTATION_VERTICAL_ON_HEAD:
            Serial.println(F("ORIENTATION_VERTICAL_ON_HEAD"));
            break;

        case ORIENTATION_HORIZONTAL:
            Serial.println(F("ORIENTATION_HORIZONTAL"));
            break;

        default:
            break;
        }

        oldOrientation = gOrientationState;
    }

    if (oldRotation != gRotationState)
    {
        switch (gRotationState)
        {
        case ROTATION_NO:
            Serial.println(F("ROTATION_NO"));
            break;

        case ROTATION_POSITIVE:
            Serial.println(F("ROTATION_POSITIVE"));
            break;

        case ROTATION_NEGATIVE:
            Serial.println(F("ROTATION_NEGATIVE"));
            break;

        default:
            break;
        }

        oldRotation = gRotationState;
    }
}

#endif /* DEBUG */