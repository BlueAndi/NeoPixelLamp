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
 * @brief  Mode: Knight Rider
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ModeKnightRider.h"
#include <Board.h>
#include <Constants.h>

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
 * This class handles mode Knight Rider within a state machine.
 * It shows colors like in the film serie "Knight Rider" in the car front.
 */
class ModeKnightRiderState : public IState
{
public:
    /**
     * Construct the mode.
     */
    ModeKnightRiderState() :
        IState(),
        m_colorWheelIndex(0U),
        m_oldPosition(Constants::neoPixelStripLength - 1U),
        m_retPosition(Constants::neoPixelStripLength - 1U),
        m_direction(DIRECTION_FORWARD)
    {
    }

    /**
     * Destroy the mode.
     */
    ~ModeKnightRiderState()
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
    /** Direction of the Knight Rider light. */
    enum Direction
    {
        DIRECTION_FORWARD = 0, /**< Forward to higher pixel id. */
        DIRECTION_BACKWARD     /**< Backward to lower pixel id. */

    };

    uint8_t   m_colorWheelIndex; /**< Color wheel index. */
    uint8_t   m_oldPosition;     /**< Old LED position. */
    uint8_t   m_retPosition;     /**< Return position, where the direction changes. */
    Direction m_direction;       /**< Direction */

    /** Copy constructor now allowed. */
    ModeKnightRiderState(const ModeKnightRiderState& other);

    /** Assignment operator now allowed. */
    ModeKnightRiderState& operator=(const ModeKnightRiderState& other);
};

void ModeKnightRiderState::entry()
{
    /* Nothing to do. */
}

void ModeKnightRiderState::process(StateMachine& sm)
{
    Adafruit_NeoPixel& neoPixel      = Board::getInstance().getPixelDrv();
    uint8_t            pos           = 0U;
    const uint8_t      MAX_POS_INDEX = Constants::neoPixelStripLength - 1U;
    const uint32_t     BLACK         = 0U;

    if (DIRECTION_FORWARD == m_direction)
    {
        if (MAX_POS_INDEX <= m_oldPosition)
        {
            pos = 0U;
        }
        else
        {
            pos = m_oldPosition + 1U;
        }

        /* If the LED reached the return position, change the direction. */
        if (m_retPosition == pos)
        {
            m_direction = DIRECTION_BACKWARD;
        }
    }
    else
    {
        if (0 == m_oldPosition)
        {
            pos = MAX_POS_INDEX;
        }
        else
        {
            pos = m_oldPosition - 1U;
        }

        /* If the LED reached the return position, change the direction. */
        if (m_retPosition == pos)
        {
            m_direction = DIRECTION_FORWARD;

            /* Rotate the LED return position. */
            if (0 == m_retPosition)
            {
                m_retPosition = MAX_POS_INDEX;
            }
            else
            {
                --m_retPosition;
            }
        }
    }

    neoPixel.setPixelColor(m_oldPosition, BLACK);
    neoPixel.setPixelColor(pos, ColorWheel::turn(m_colorWheelIndex));
    neoPixel.show();

    ++m_colorWheelIndex; /* Wrap around by intention. */
    m_oldPosition = pos;
}

void ModeKnightRiderState::exit()
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
static ModeKnightRiderState gMode;

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

IState* ModeKnightRider::getState()
{
    return &gMode;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
