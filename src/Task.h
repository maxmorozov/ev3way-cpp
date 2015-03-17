/*
 * Task.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef TASK_H_
#define TASK_H_

#include <chrono>
#include <thread>
#include <functional>

namespace ev3way {

class Task {
public:
	typedef std::chrono::system_clock clock_type;
private:
	clock_type::duration m_period;
	std::function<void()> m_task;
	std::function<bool()> m_stopChecker;

public:
    template<typename T, typename F>
	Task(const clock_type::duration& period, const T& task, F stopper)
		: m_period(period), m_task(task), m_stopChecker(stopper)
	{
	}

	void operator()() {
	    clock_type::time_point next_stop = clock_type::now();

	    while (!m_stopChecker()) {
	    	m_task();

	        next_stop += m_period;

	        std::this_thread::sleep_until(next_stop);
	    }
	}
};

} /* namespace ev3way */

#endif /* TASK_H_ */
