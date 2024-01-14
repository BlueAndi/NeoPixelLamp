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
 * @brief  Mode: Color
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ModeColor.h"
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
 * It shows the user selected color (turn the lamp to choose).
 */
class ModeColorState : public IState
{
public:
    /**
     * Construct the mode.
     */
    ModeColorState() : IState(), m_colorWheelIndex(0U)
    {
    }

    /**
     * Destroy the mode.
     */
    ~ModeColorState()
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
    ModeColorState(const ModeColorState& other);

    /** Assignment operator now allowed. */
    ModeColorState& operator=(const ModeColorState& other);

    /**
     * Show colors on pixel strip.
     */
    void show();
};

void ModeColorState::entry()
{
    show();
}

void ModeColorState::process(StateMachine& sm)
{
    Board&           board   = Board::getInstance();
    AccelerationDrv& accDrv  = board.getAccelerationSensor();
    GyroscopeDrv&    gyroDrv = board.getGyroscopeSensor();
    sensors_vec_t    gyro;
    sensors_vec_t    acceleration;
    bool             hasChanged                    = false;
    const float      LAMP_TAIL_ABS_ACC_Z_THRESHOLD = 8.0F;
    const float      LAMP_TURN_GYRO_Z_THRESHOLD    = 20.0F;

    accDrv.getAcceleration(acceleration);
    gyroDrv.getGyro(gyro);

    /* The lamp shall stand on the tail. */
    if (LAMP_TAIL_ABS_ACC_Z_THRESHOLD < fabs(acceleration.z))
    {
        /* If the lamp is turned right, the color wheel position increases. */
        if (LAMP_TURN_GYRO_Z_THRESHOLD < gyro.z)
        {
            ++m_colorWheelIndex; /* Wrap around by intention. */
            hasChanged = true;
        }
        /* If the lamp is turned left, the color wheel position decreases */
        else if (-LAMP_TURN_GYRO_Z_THRESHOLD > gyro.z)
        {
            --m_colorWheelIndex; /* Wrap around by intention. */
            hasChanged = true;
        }
        /* Nothing to do */
        else
        {
            ;
        }
    }

    if (true == hasChanged)
    {
        show();
    }
}

void ModeColorState::exit()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();

    neoPixel.clear();
    neoPixel.show();
}

void ModeColorState::show()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();
    uint8_t            run;

    for (run = 0; run < neoPixel.numPixels(); ++run)
    {
        /* Turn every third pixel on */
        neoPixel.setPixelColor(run, ColorWheel::turn(m_colorWheelIndex));
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
static ModeColorState gMode;

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

IState* ModeColor::getState()
{
    return &gMode;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
