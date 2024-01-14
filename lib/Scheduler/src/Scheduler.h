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
 * @brief  Simple cooperative task scheduler
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup OS
 *
 * @{
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include "TaskBase.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Cooperative task scheduler.
 *
 * Functionality:
 * - The scheduler will get a constant array of tasks, which to schedule at compile time.
 * - Every active task will be called once during one execution cycle.
 * - Tasks which are suspended are not called.
 * - Tasks which are waiting for a signal, will be called if the signal is signalling.
 */
class Scheduler
{
public:
    /** Construct the instance. */
    Scheduler(TaskBase** taskList, uint8_t count) : m_taskList(taskList), m_count(count)
    {
    }

    /** Destroy the instance. */
    ~Scheduler()
    {
    }

    /**
     * Process all tasks in the scheduler.
     */
    void execute();

private:
    TaskBase** m_taskList; /**< List of tasks which to schedule. */
    uint8_t    m_count;    /**< Number of tasks in the list. */

    /** Don't want a default constructor. */
    Scheduler();

    /** Don't want a copy constructor. */
    Scheduler(const Scheduler& scheduler);

    /** Don't want a assignment operator. */
    const Scheduler& operator=(const Scheduler& scheduler);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SCHEDULER_H */
/** @} */
