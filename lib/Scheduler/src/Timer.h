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
 * @brief  Simple timer
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup OS
 *
 * @{
 */

#ifndef TIMER_H
#define TIMER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "ISignal.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class is a simple timer. */
class Timer : public ISignal
{
public:
    /**
     * Prototype of the function, which all timers are calling to retrieve the
     * current timestamp in ms.
     *
     * @return Timestamp in ms since program start.
     */
    typedef uint32_t (*GetTimestampFunc)();

    /**
     * Construct the instance.
     */
    Timer() : m_isRunning(false), m_isTimeout(false), m_duration(0U), m_startTimestamp(0U)
    {
    }

    /**
     * Construct the instance.
     *
     * @param[in] duration  Timer duration in ms.
     */
    Timer(uint32_t duration) : m_isRunning(false), m_isTimeout(false), m_duration(duration), m_startTimestamp(0U)
    {
    }

    /**
     * Construct the instance by copy.
     *
     * @param[in] timer The timer to be copied.
     */
    Timer(const Timer& timer) :
        m_isRunning(timer.m_isRunning),
        m_isTimeout(timer.m_isTimeout),
        m_duration(timer.m_duration),
        m_startTimestamp(timer.m_startTimestamp)
    {
    }

    /**
     * Assigns a timer.
     *
     * @param[in] timer The timer to be assigned.
     */
    Timer& operator=(const Timer& timer)
    {
        /* Avoid self-assignment */
        if (&timer != this)
        {
            m_isRunning      = timer.m_isRunning;
            m_isTimeout      = timer.m_isTimeout;
            m_duration       = timer.m_duration;
            m_startTimestamp = timer.m_startTimestamp;
        }

        return *this;
    }

    /**
     * Destroy the instance.
     */
    ~Timer()
    {
    }

    /**
     * Start timer with the given duration in ms.
     *
     * @param[in] duration Duration in ms
     */
    void start(uint32_t duration);

    /**
     * Restart timer with the same duration as the timer was started before.
     */
    void restart();

    /**
     * Stop timer.
     */
    void stop();

    /**
     * Is timer running?
     *
     * @return If timer is running, it will return true otherwise false.
     */
    bool isRunning() const;

    /**
     * Is timeout?
     * Note, if the timer is not started the method will return false.
     *
     * @return If timeout it will return true otherwise false.
     */
    bool isTimeout();

    /**
     * Get current duration in ms, till the timer was started.
     * It is independed of whether the timer is stopped or timeout.
     */
    uint32_t getCurrentDuration() const;

    /**
     * Get the signal state.
     * If the timer is running and expired, it will be automatically restarted.
     *
     * @return  Signal state
     * @retval  false   Non-signaled
     * @retval  true    Signaled
     */
    bool isSignaled(void) override
    {
        bool status = isTimeout();

        if (true == status)
        {
            restart();
        }

        return status;
    }

    /**
     * Initialize the timer functionality.
     * This shall be done before any timer is used!
     *
     * @param[in] getTimestampFunc  Function to retrieve current timestamp.
     */
    static void init(GetTimestampFunc getTimestampFunc)
    {
        m_getTimestampFunc = getTimestampFunc;
    }

private:
    static GetTimestampFunc m_getTimestampFunc; /**< Function to retrieve the current timestamp. */

    bool     m_isRunning;      /**< Is timer running (true) or not (false). */
    bool     m_isTimeout;      /**< Timeout flag */
    uint32_t m_duration;       /**< Duration in ms */
    uint32_t m_startTimestamp; /**< Timestamp in ms at start. */

    /**
     * Get timestamp in ms.
     *
     * @return Timestamp in ms since program start.
     */
    uint32_t getTimestamp() const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TIMER_H */
/** @} */
