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
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <TaskBase.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * This type defines the different orientation states.
 * That means how the lamp is oriented in relation to earth ground.
 */
typedef enum
{
    ORIENTATION_UNKNOWN = 0,        /**< Unknown orientation */
    ORIENTATION_VERTICAL_ON_BOTTOM, /**< Vertical orientation on bottom */
    ORIENTATION_VERTICAL_ON_HEAD,   /** Vertical orientation on head */
    ORIENTATION_HORIZONTAL          /**< Horizontal orientation */

} Orientation;

/** This type defines the different rotation states. */
typedef enum
{
    ROTATION_NO = 0,   /**< No rotation */
    ROTATION_POSITIVE, /**< Positive rotation */
    ROTATION_NEGATIVE  /**< Negative Rotation */

} Rotation;

/******************************************************************************
 * Functions
 *****************************************************************************/

/**
 * Motion task related.
 */
namespace TaskMotion
{
    /**
     * Get the task.
     *
     * @return Task
     */
    TaskBase* getTask();

    /**
     * Get lamp orientation state.
     *
     * @return Orientation
     */
    Orientation getOrientation();

    /**
     * Get lamp rotation state.
     *
     * @return Rotation
     */
    Rotation getRotation();

    /**
     * Is a lamp shake detected?
     * 
     * @return If lamp shake is detected, it will return true otherwise false.
     */
    bool isShakeDetected();

}; /* namespace TaskMotion */

#endif /* TASK_MOTION_H */
/** @} */
