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
 * @brief  Cooperative base task
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup OS
 *
 * @{
 */

#ifndef TASK_BASE_H
#define TASK_BASE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdlib.h>
#include "ISignal.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * This base class defines a task which will be executed by the scheduler.
 *
 * How to use? Derive from it and implement the process() method.
 */
class TaskBase
{
public:
    /** Task states */
    enum STATE
    {
        STATE_SUSPENDED = 0, /**< Task is suspended. */
        STATE_RUNNING,       /**< Task is running. */
        STATE_WAITING        /**< Task is waiting for a resource. */
    };

    /**
     * Task function which is defined by the user.
     */
    typedef void (*TaskFunc)(void* par);

    /**
     * Construct the instance.
     *
     * @param[in] suspended Shall the task be suspended or running at the begin? Default: suspended
     */
    TaskBase(bool suspended = true) :
        m_state((true == suspended) ? (STATE_SUSPENDED) : (STATE_RUNNING)),
        m_prevState(m_state),
        m_signal(NULL)
    {
    }

    /** Destroy the instance. */
    virtual ~TaskBase()
    {
    }

    /**
     * Execute the task depended on its internal state.
     * It will be executed by the scheduler.
     */
    void execute(void);

    /**
     * Suspend the task.
     */
    void suspend(void)
    {
        if (STATE_SUSPENDED != m_state)
        {
            m_prevState = m_state;
            m_state = STATE_SUSPENDED;
        }
    }

    /**
     * Resume the task. It will be in the state before it was suspended.
     */
    void resume(void)
    {
        m_state = m_prevState;
    }

    /**
     * Get the current task state.
     *
     * @return  Task state
     */
    STATE getState(void) const
    {
        return m_state;
    }

    /**
     * Let the task wait for a signal event. Every time the event is signaled,
     * the task will run.
     *
     * @param[in] sig   Signal event to wait for.
     */
    void waitFor(ISignal& sig)
    {
        m_state  = STATE_WAITING;
        m_signal = &sig;
    }

private:
    STATE    m_state;     /**< Current task state. */
    STATE    m_prevState; /**< Previous task state. */
    ISignal* m_signal;    /**< Signal event to wait for. */

    /** Don't want a copy constructor */
    TaskBase(const TaskBase& task);

    /** Don't want a assignment operator */
    TaskBase& operator=(const TaskBase& task);

    /**
     * Process the task function.
     */
    virtual void process(void) = 0;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* TASK_BASE_H */
/** @} */
