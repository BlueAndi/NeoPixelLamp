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
 * @brief  Timer triggered cooperative task
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup OS
 *
 * @{
 */

#ifndef TTASK_H
#define TTASK_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "TaskBase.h"
#include "Timer.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Timer triggered task which process the user defined function.
 * Its not necessary to derived from it! If a class method shall be called
 * by the task, derive your class from the TaskBase.
 */
class TTask : public TaskBase
{
public:
    /**
     * Construct the instance.
     *
     * @param[in] taskFunc      Task function, which shall be called by the scheduler.
     * @param[in] taskFuncPar   Task function parameter.
     * @param[in] period        Task period in ms.
     * @param[in] suspended     Shall the task be suspended or running at the begin? Default: suspended
     */
    TTask(TaskFunc taskFunc, void* taskFuncPar, uint32_t period, bool suspended = true) :
        TaskBase(suspended),
        m_taskFunc(taskFunc),
        m_taskFuncPar(taskFuncPar),
        m_timer(period)
    {
        restart();
    }

    /**
     * Destroy the instance.
     */
    virtual ~TTask()
    {
    }

    /**
     * Restart timer triggered task.
     */
    void restart()
    {
        /* Use the timer to run the task periodically. */
        waitFor(m_timer);
        m_timer.restart();
    }

    /**
     * Change call period.
     * 
     * @param[in] period New period in ms which to set.
     */
    void setPeriod(uint32_t period)
    {
        m_timer.start(period);
    }

private:
    TaskFunc m_taskFunc;    /**< User specific function which to call. */
    void*    m_taskFuncPar; /**< User function specific parameter. */
    Timer    m_timer;       /**< Timer used to execute the task periodically. */

    /**
     * Process the task function.
     */
    void process(void) override
    {
        if (nullptr != m_taskFunc)
        {
            m_taskFunc(m_taskFuncPar);
        }
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TTASK_H */
/** @} */
