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
 * @brief  Mode handling task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskModeHandler.h"
#include <TTask.h>
#include <StateMachine.h>
#include <Board.h>
#include <Constants.h>

#include "TaskMotion.h"
#include "ModeRainbow.h"
#include "ModeTheatre.h"
#include "ModeTemperature.h"
#include "ModeColor.h"
#include "ModeKnightRider.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Get number of elements in the given array. */
#define ARRAY_NUM(__arr) (sizeof(__arr) / sizeof((__arr)[0]))

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/**
 * Mode related data.
 */
typedef struct
{
    const char* name;   /**< Name of the mode. */
    IState*     state;  /**< State which handles the mode. */
    uint32_t    period; /**< Call period of the state. */

} Mode;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void modeHandler(void* par);
void        selectNextMode();
static void redSplash(uint8_t count);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task for mode handling. */
static TTask gTask(modeHandler, nullptr, 20U, false);

/** Each mode is handled by a separate state. */
static StateMachine gStateMachine;

/** Mode name: Rainbow */
static const char PROGMEM gModeNameRainbow[] = "Rainbow";

/** Mode name: Theatre */
static const char PROGMEM gModeNameTheatre[] = "Theatre";

/** Mode name: Temperature */
static const char PROGMEM gModeNameTemperature[] = "Temperature";

/** Mode name: Color */
static const char PROGMEM gModeNameColor[] = "Color";

/** Mode name: KnightRider */
static const char PROGMEM gModeNameKnightRider[] = "KnightRider";

/** List of modes. The order is used in case the user selects the next mode. */
static Mode gModeList[] = {{gModeNameRainbow, ModeRainbow::getState(), 20U},
                           {gModeNameTheatre, ModeTheatre::getState(), 100U},
                           {gModeNameTemperature, ModeTemperature::getState(), 100U},
                           {gModeNameColor, ModeColor::getState(), 20U},
                           {gModeNameKnightRider, ModeKnightRider::getState(), 20U}};

/** Index of the current mode in the mode list. */
static size_t gIndexOfCurrentMode = 0U;

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

TaskBase* TaskModeHandler::getTask()
{
    return &gTask;
}

void TaskModeHandler::restartMode()
{
    IState* currentState = gStateMachine.getState();

    if (nullptr != currentState)
    {
        currentState->entry();
    }
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Task function which handles the current mode and mode changes.
 *
 * @param[in] par Task parameter
 */
static void modeHandler(void* par)
{
    (void)par;

    /* If no mode is active, set the current selected one. */
    if (nullptr == gStateMachine.getState())
    {
        gStateMachine.setState(gModeList[gIndexOfCurrentMode].state);
    }

    if (true == TaskMotion::isShakeDetected())
    {
        selectNextMode();
    }

    /** Execute the current mode. */
    gStateMachine.process();
}

/**
 * Select the next mode.
 */
void selectNextMode()
{
    /* Select next mode. */
    ++gIndexOfCurrentMode;

    /* Handle wrap around. */
    if (ARRAY_NUM(gModeList) <= gIndexOfCurrentMode)
    {
        gIndexOfCurrentMode = 0U;
    }

    Serial.print(F("Change to mode "));
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(gModeList[gIndexOfCurrentMode].name));

    /* Choose the state which handles the new mode. */
    gStateMachine.setState(gModeList[gIndexOfCurrentMode].state);
    gTask.setPeriod(gModeList[gIndexOfCurrentMode].period);

    /* Notify user about mode change. */
    redSplash(gIndexOfCurrentMode);
}

/**
 * A red LED splash.
 *
 * @param[in] count Number of splashes
 */
static void redSplash(uint8_t count)
{
    Adafruit_NeoPixel& neoPixel      = Board::getInstance().getPixelDrv();
    uint8_t            counted       = 0U;
    uint8_t            oldBrightness = neoPixel.getBrightness(); /* Store current brightness. */
    const uint32_t     RED           = neoPixel.Color(255U, 0U, 0U);

    neoPixel.clear();
    neoPixel.show();
    delay(200U); /* Not nice, but simple. */

    neoPixel.setBrightness(Constants::neoPixelMaxBrightness / 2U);

    while (counted < count)
    {
        uint16_t run;

        for (run = 0; run < neoPixel.numPixels(); ++run)
        {
            neoPixel.setPixelColor(run, RED);
            neoPixel.show();
        }

        delay(500U); /* Not nice, but simple. */

        neoPixel.clear();
        neoPixel.show();
        delay(500U); /* Not nice, but simple. */

        ++counted;
    }

    /* Restore brightness */
    neoPixel.setBrightness(oldBrightness);
}
