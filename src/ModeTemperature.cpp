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
 * @brief  Mode: Temperature
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ModeTemperature.h"
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
 * This class handles mode Theater within a state machine.
 * It shows the color dependend on the measured temperature.
 */
class ModeTemperatureState : public IState
{
public:
    /**
     * Construct the mode.
     */
    ModeTemperatureState() : IState(), m_lastTemperature(0.0F), m_temperatures{0}, m_wrIndex(0U)
    {
    }

    /**
     * Destroy the mode.
     */
    ~ModeTemperatureState()
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
    /** Number of temperatures over the average temperature is calculated. */
    static const size_t AVERAGE_COUNT = 10U;

    /** Colors used for temperature depended color change. */
    static const uint32_t COLORS[];

    float   m_lastTemperature;             /**< Last temperature in °C. */
    int8_t  m_temperatures[AVERAGE_COUNT]; /**< Temperatures used for average calculation. */
    uint8_t m_wrIndex;                     /**< Write index in the temperature array. */

    ModeTemperatureState(const ModeTemperatureState& other);
    ModeTemperatureState& operator=(const ModeTemperatureState& other);

    /**
     * Get temperature in °C as fixpoint integer and limited by
     * supported temperature range.
     *
     * @return Temperature in °C
     */
    int8_t getTemperature();

    /**
     * Add temperature to temperature array.
     *
     * @param[in] temperature   Temperature in °C
     */
    void addTemperature(int8_t temperature);

    /**
     * Get temperature average in °C.
     *
     * @return Temperature average in °C
     */
    float getTemperatureAvg();
};

const uint32_t ModeTemperatureState::COLORS[] = {
    Adafruit_NeoPixel::Color(255U, 0U, 255U), /* Purple */
    Adafruit_NeoPixel::Color(0U, 0U, 255U),   /* Blue */
    Adafruit_NeoPixel::Color(0U, 255U, 255U), /* Cyan */
    Adafruit_NeoPixel::Color(0U, 255U, 0U),   /* Green */
    Adafruit_NeoPixel::Color(255U, 255U, 0U), /* Yellow */
    Adafruit_NeoPixel::Color(255U, 0U, 0U)    /* Red */
};

void ModeTemperatureState::entry()
{
    int8_t  temperature = getTemperature();
    uint8_t index;

    /* Initialize temperature array with current temperature. */
    for (index = 0U; index < AVERAGE_COUNT; ++index)
    {
        m_temperatures[index] = temperature;
    }

    /* Force color update by exceeding the max. temperature. */
    m_lastTemperature = Constants::temperatureMax + 10.0F;
}

void ModeTemperatureState::process(StateMachine& sm)
{
    Board&             board                      = Board::getInstance();
    Adafruit_NeoPixel& neoPixel                   = board.getPixelDrv();
    int8_t             temperature                = getTemperature(); /* [°C] */
    float              temperatureAvg             = 0.0F; /* [°C] */
    const float        TEMPERATURE_DIFF_THRESHOLD = 0.1F; /* [°C] */

    addTemperature(temperature);
    temperatureAvg = getTemperatureAvg();

    if (TEMPERATURE_DIFF_THRESHOLD <= fabs(m_lastTemperature - temperatureAvg))
    {
        uint16_t run;
        float    fNormalizedTemperature = (temperatureAvg - Constants::temperatureMin) /
                                       (Constants::temperatureMax - Constants::temperatureMin); /* [°C] */
        float    fColorIndex    = fNormalizedTemperature * static_cast<float>(AVERAGE_COUNT - 1U);
        uint8_t  colorIndexHigh = static_cast<uint8_t>(ceilf(fColorIndex));         /* Round up */
        uint8_t  colorIndexLow  = static_cast<uint8_t>(floorf(fColorIndex));        /* Round down */
        float    dx             = static_cast<float>(colorIndexHigh) - fColorIndex; /* Calculate color index distance */
        uint32_t colorHigh      = COLORS[colorIndexHigh];
        uint32_t colorLow       = COLORS[colorIndexLow];
        uint8_t  colorHighRed   = (colorHigh >> 16U) & 0xFFU;
        uint8_t  colorLowRed    = (colorLow >> 16U) & 0xFFU;
        uint8_t  colorHighGreen = (colorHigh >> 8U) & 0xFFU;
        uint8_t  colorLowGreen  = (colorLow >> 8U) & 0xFFU;
        uint8_t  colorHighBlue  = (colorHigh >> 0U) & 0xFFU;
        uint8_t  colorLowBlue   = (colorLow >> 0U) & 0xFFU;
        uint8_t  red            = static_cast<uint8_t>(static_cast<float>(colorHighRed) * (1.0F - dx) +
                                           static_cast<float>(colorLowRed) * dx); /* Interpolate */
        uint8_t  green          = static_cast<uint8_t>(static_cast<float>(colorHighGreen) * (1.0F - dx) +
                                             static_cast<float>(colorLowGreen) * dx); /* Interpolate */
        uint8_t  blue           = static_cast<uint8_t>(static_cast<float>(colorHighBlue) * (1.0F - dx) +
                                            static_cast<float>(colorLowBlue) * dx); /* Interpolate */
        uint32_t color          = Adafruit_NeoPixel::Color(red, green, blue);

        Serial.print("Temperature: ");
        Serial.print(temperatureAvg, 1);
        Serial.println(" *C");
        Serial.print("RGB        : ");
        Serial.print(red);
        Serial.print(" ");
        Serial.print(green);
        Serial.print(" ");
        Serial.println(blue);

        for (run = 0U; run < neoPixel.numPixels(); ++run)
        {
            neoPixel.setPixelColor(run, color);
            neoPixel.show();
        }

        m_lastTemperature = temperatureAvg;
    }
}

void ModeTemperatureState::exit()
{
    Adafruit_NeoPixel& neoPixel = Board::getInstance().getPixelDrv();

    neoPixel.clear();
    neoPixel.show();
}

int8_t ModeTemperatureState::getTemperature()
{
    Board&          board          = Board::getInstance();
    TemperatureDrv& temperatureDrv = board.getTemperatureSensor();
    float           fTemperature   = temperatureDrv.getTemperature();
    int8_t          temperature;

    /* Determine fixpoint temperature. */
    if (Constants::temperatureMin > fTemperature)
    {
        temperature = static_cast<int8_t>(Constants::temperatureMin);
    }
    else if (Constants::temperatureMax < fTemperature)
    {
        temperature = static_cast<int8_t>(Constants::temperatureMax);
    }
    else
    {
        temperature = static_cast<int8_t>(fTemperature);
    }

    return temperature;
}

void ModeTemperatureState::addTemperature(int8_t temperature)
{
    m_temperatures[m_wrIndex] = temperature;
    ++m_wrIndex;

    /* Wrap around? */
    if (AVERAGE_COUNT <= m_wrIndex)
    {
        m_wrIndex = 0U;
    }
}

float ModeTemperatureState::getTemperatureAvg()
{
    uint8_t index;
    float   average = 0.0F;

    /* Calculate temperature average. */
    for (index = 0; index < AVERAGE_COUNT; ++index)
    {
        average += static_cast<float>(m_temperatures[index]);
    }

    average /= static_cast<float>(AVERAGE_COUNT);

    return average;
}

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Mode instance. */
static ModeTemperatureState gMode;

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

IState* ModeTemperature::getState()
{
    return &gMode;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
