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
 * @brief  Mode: Theater
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ModeTheater.h"
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
 * This class handles mode Theater within a state machine.
 * It shows theater-style crawling lights with rainbow effect.
 */
class ModeTheaterState : public IState
{
public:
    /**
     * Construct the mode.
     */
    ModeTheaterState() : IState(), m_colorWheelIndex(0U), m_onIndex(0U)
    {
    }

    /**
     * Destroy the mode.
     */
    ~ModeTheaterState()
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
    /** Every third LED is on. */
    static const uint8_t DISTANCE = 3U;

    uint8_t m_colorWheelIndex; /**< Color wheel index. */
    uint8_t m_onIndex;         /**< Index decides whether LED is on. */

    /** Copy constructor now allowed. */
    ModeTheaterState(const ModeTheaterState& other);

    /** Assignment operator now allowed. */
    ModeTheaterState& operator=(const ModeTheaterState& other);

    /**
     * Show colors on pixel strip.
     */
    void show();
};

void ModeTheaterState::entry()
{
    show();
}

void ModeTheaterState::process(StateMachine& sm)
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();
    uint8_t            run;
    const uint32_t     BLACK = 0U;

    /* Switch off current LEDs off, which are on. */
    for (run = 0; run < neoPixel.numPixels(); run += DISTANCE)
    {
        /* Turn every third pixel off */
        neoPixel.setPixelColor(run + m_onIndex, BLACK);
    }

    if ((DISTANCE - 1U) <= m_onIndex)
    {
        m_onIndex = 0U;

        ++m_colorWheelIndex; /* Wrap around by intention. */
    }
    else
    {
        ++m_onIndex;
    }

    show();
}

void ModeTheaterState::exit()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();

    neoPixel.clear();
    neoPixel.show();
}

void ModeTheaterState::show()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();
    uint8_t            run;

    for (run = 0; run < neoPixel.numPixels(); run += DISTANCE)
    {
        /* Turn every third pixel on */
        neoPixel.setPixelColor(run + m_onIndex, ColorWheel::turn(run + m_colorWheelIndex));
    }

    neoPixel.show();
}

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Mode instance. */
static ModeTheaterState gMode;

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

IState* ModeTheater::getState()
{
    return &gMode;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
