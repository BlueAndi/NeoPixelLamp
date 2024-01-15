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
 * @brief  Main entry point
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <Board.h>
#include <Scheduler.h>
#include <Task.h>
#include <TTask.h>

#include "TaskModeHandler.h"
#include "TaskUI.h"
#include "TaskSleep.h"
#include "TaskDebugPlot.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/**
 * Enables only the debug plot task. It will provide values over the serial
 * interface to track them over the serial plotter.
 */
#define ENABLE_DEBUG_PLOT 0

/**
 * For debug purposes it is better to wait for an established USB connection (1)
 * otherwise nothing will be seen in the serial console.
 * In normal mode (no debugging) without a USB connection, it must be disabled,
 * otherwise the program will hang in a infinite loop.
 */
#define NEOPIXELLAMP_WAIT_FOR_USB 0

/** Get number of elements in the given array. */
#define ARRAY_NUM(__arr) (sizeof(__arr) / sizeof((__arr)[0]))

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static uint32_t getTimestamp();
static void     loadCalibration();

/******************************************************************************
 * Variables
 *****************************************************************************/

#if (0 == ENABLE_DEBUG_PLOT)

/** The list of tasks which are scheduled by the scheduler. */
static TaskBase* gTaskList[] = {
    TaskModeHandler::getTask(), /* Handles the selected mode. */
    TaskUI::getTask(),          /* Handles the user interface. */
    TaskSleep::getTask()        /* Handles the sleep mode. */
};

#else /* (0 == ENABLE_DEBUG_PLOT) */

/** The list of tasks which are scheduled by the scheduler. */
static TaskBase* gTaskList[] = {TaskDebugPlot::getTask()};

#endif /* (0 == ENABLE_DEBUG_PLOT) */

/** Serial baudrate. */
static const unsigned long SERIAL_BAUDRATE = 9600U;

/** The task scheduler. */
static Scheduler gScheduler(gTaskList, ARRAY_NUM(gTaskList));

/******************************************************************************
 * External functions
 *****************************************************************************/

/**
 * Initialize the system.
 * This function is called once during startup.
 */
void setup() /* cppcheck-suppress unusedFunction */
{
    Board& board = Board::getInstance();

    /* Set serial baudrate */
    Serial.begin(SERIAL_BAUDRATE);

#if (0 != NEOPIXELLAMP_WAIT_FOR_USB)

    /* Wait for serial port to connect. Needed for native USB */
    while (!Serial)
        ;

#endif

    Serial.println("Setup NeoPixelLamp ...");

    if (false == board.init())
    {
        Serial.println("Failed to initialize the hardware.");
        delay(100U); /* Ensure that the previous printed info is transferred over serial connection. */

        Board::reset();
    }
    else
    {
        loadCalibration();

        Serial.println("NeoPixelLamp is ready.");
    }

    Timer::init(getTimestamp);
}

/**
 * Main program loop.
 * This function is called cyclic.
 */
void loop() /* cppcheck-suppress unusedFunction */
{
    gScheduler.execute();
}

/******************************************************************************
 * Local functions
 *****************************************************************************/

/**
 * Get current timestamp in ms.
 *
 * @return Timestamp in ms
 */
static uint32_t getTimestamp()
{
    return static_cast<uint32_t>(millis());
}

/**
 * Load the calibration values for the magnetometer.
 */
static void loadCalibration()
{
    Board&           board    = Board::getInstance();
    AccelerationDrv& accDrv   = board.getAccelerationSensor();
    Settings&        settings = board.getSettings();
    sensors_vec_t    offset;

    offset.x = settings.getAccelerationXOffset();
    offset.y = settings.getAccelerationYOffset();
    offset.z = settings.getAccelerationZOffset();

    accDrv.setOffset(offset);
    accDrv.enableOffsetCompensation();
}