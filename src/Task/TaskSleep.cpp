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
 * @brief  Sleep mode task
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskSleep.h"
#include <Task.h>
#include <Board.h>
#include <Constants.h>
#include "TaskModeHandler.h"
#include "TaskBrightness.h"
#include "TaskMotion.h"

#include <avr/sleep.h>
#include <avr/wdt.h>

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
static void fadeOut();

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Task which handles the system sleep to reduce power consumption. */
static Task gTask(taskFunc, nullptr, false);

/**
 * Flag to indicate whether sleep mode is active or not.
 * Use it during the watchdog wake up's to know whether sleep mode is already
 * active or not.
 */
static bool gSleepModeActive = false;

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

TaskBase* TaskSleep::getTask()
{
    return &gTask;
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * System enters sleep mode to reduce power consumption.
 *
 * @param[in] par   Task parameter
 */
static void taskFunc(void* par)
{
    Board&           board  = Board::getInstance();
    AccelerationDrv& accDrv = board.getAccelerationSensor();
    sensors_vec_t    acceleration;
    const float      MAX_WAKE_UP_ACC_Z = -Constants::earthGravity + 1.0F; /* [m/s^2] */
    const float      MIN_WAKE_UP_ACC_Z = -Constants::earthGravity - 1.0F; /* [m/s^2] */

    accDrv.getAcceleration(acceleration);

    /* If the lamp is on its head, the user requests sleep mode. */
    if ((MAX_WAKE_UP_ACC_Z >= acceleration.z) && (MIN_WAKE_UP_ACC_Z <= acceleration.z))
    {
        if (false == gSleepModeActive)
        {
            Serial.println(F("Enter sleep mode."));

            /* Stop animations. */
            TaskModeHandler::getTask()->suspend();
            TaskBrightness::getTask()->suspend();
            fadeOut();

            /* Ensure the onboard LED is off to save power. */
            Board::getInstance().ledOff();

            USBDevice.detach();

            gSleepModeActive = true;
        }

#if 0
        // Power down the LSM9DS0 sensor
        uint8_t regCtrlReg1G = lsm.read8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G);
        regCtrlReg1G &= ~(0x08);
        lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, regCtrlReg1G);
#endif

        /* Configure sleep mode power down. */
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();

        /* Enable watchdog
         * Watchdog timer          : 1 s
         * Watchdog timer interrupt: enabled
         * Watchdog reset          : disabled
         */
        {
            cli();

            wdt_reset();

            /* Prepare watchdog for timer interrupt without reset. */
            MCUSR &= ~(1 << WDRF); /* Clear WDRF, which overwrites WDE. */

            /* In the same operation, write a logic one to the Watchdog
             * change enable bit (WDCE) and WDE. A logic one must be
             * written to WDE regardless of the previous value of the WDE bit.
             */
            WDTCSR |= (1 << WDCE) | (1 << WDE);

            /* Within the next four clock cycles, write the WDE and Watchdog
             * prescaler bits (WDP) as desired, but with the WDCE bit cleared.
             * This must be done in one operation.
             * Watchdog timer: 1 s
             * Watchdog interrupt mode
             */
            WDTCSR = (1 << WDIE) | (0 << WDCE) | (0 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);

            sei();
        }

        /* Sleep now! */
        sleep_mode();

        /* Waked up again ... */
        sleep_disable();

#if 0
        // Power up the LSM9DS0 sensor
        regCtrlReg1G |= 0x08;
        lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, regCtrlReg1G);
#endif
    }
    else
    {
        if (true == gSleepModeActive)
        {
            USBDevice.attach();

            Serial.println(F("Leave sleep mode."));

            /* Play animations again. */
            TaskModeHandler::getTask()->resume();
            TaskBrightness::getTask()->resume();

            /* Mode must be restarted! */
            TaskModeHandler::restartMode();

            gSleepModeActive = false;
        }
    }
}

/**
 * Fade pixels out.
 */
static void fadeOut()
{
    Board&             board         = Board::getInstance();
    Adafruit_NeoPixel& neoPixel      = board.getPixelDrv();
    uint8_t            oldBrightness = neoPixel.getBrightness();
    const uint32_t     RED           = Adafruit_NeoPixel::Color(255U, 0U, 0U);
    uint8_t            run;
    uint8_t            brightness;

    for (run = 0U; run < neoPixel.numPixels(); ++run)
    {
        neoPixel.setPixelColor(run, RED);
    }

    neoPixel.show();

    for (brightness = Constants::neoPixelMaxBrightness; brightness > 0U; --brightness)
    {
        neoPixel.setBrightness(brightness);
        neoPixel.show();

        delay(10U);
    }

    neoPixel.clear();
    neoPixel.show();

    /* Restore original user choosen brightness. */
    neoPixel.setBrightness(oldBrightness);
}

/**
 * Watchdog timer interrupt, used to wake up from sleep mode periodically.
 */
ISR(WDT_vect)
{
    /* Disable watchdog. */
    {
        cli();

        wdt_reset();

        /* Prepare watchdog for timer interrupt without reset. */
        MCUSR &= ~(1 << WDRF); /* Clear WDRF, which overwrites WDE. */

        /* In the same operation, write a logic one to the Watchdog
         * change enable bit (WDCE) and WDE. A logic one must be
         * written to WDE regardless of the previous value of the WDE bit.
         */
        WDTCSR |= (1 << WDCE) | (1 << WDE);

        /* Within the next four clock cycles, write the WDE and Watchdog
         * prescaler bits (WDP) as desired, but with the WDCE bit cleared.
         * This must be done in one operation.
         * Watchdog timer: disabled
         */
        WDTCSR = 0;

        sei();
    }
}

/**
 * Disable activated watchdog right after reset.
 * http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));

/** Copy of the MCUSR register. */
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void    get_mcusr(void)
{
    mcusr_mirror = MCUSR;
    MCUSR        = 0;
    wdt_disable();
}
