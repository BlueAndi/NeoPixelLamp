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
 * @brief  Mode: Rainbow
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ModeRainbow.h"
#include <Board.h>

#include "ColorWheel.h"

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
 * This class handles mode Rainbow within a state machine.
 */
class ModeRainbowState : public IState
{
public:
    /**
     * Construct the mode.
     */
    ModeRainbowState() : IState(), m_colorWheelIndex(0U)
    {
    }

    /**
     * Destroy the mode.
     */
    ~ModeRainbowState()
    {
    }

    /**
     * If the state is entered, this method will called once.
     */
    void entry() final;

    /**
     * Processing the state.
     *
     * @param[in] sm State machine, which is calling this state.
     */
    void process(StateMachine& sm) final;

    /**
     * If the state is left, this method will be called once.
     */
    void exit() final;

private:
    uint8_t m_colorWheelIndex; /**< Color wheel index. */

    /** Copy constructor now allowed. */
    ModeRainbowState(const ModeRainbowState& other);

    /** Assignment operator now allowed. */
    ModeRainbowState& operator=(const ModeRainbowState& other);
};

void ModeRainbowState::entry()
{
    /* Nothing to do. */
}

void ModeRainbowState::process(StateMachine& sm)
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();
    uint8_t           run;

    for (run = 0U; run < neoPixel.numPixels(); ++run)
    {
        neoPixel.setPixelColor(run, ColorWheel::turn(m_colorWheelIndex + run));
    }
    neoPixel.show();

    ++m_colorWheelIndex; /* Wrap around by intention. */
}

void ModeRainbowState::exit()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();

    neoPixel.clear();
    neoPixel.show();
}

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Mode instance. */
static ModeRainbowState gMode;

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

IState* ModeRainbow::getState()
{
    return &gMode;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
