/*
 * EV3Way.h
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#ifndef EV3WAY_H_
#define EV3WAY_H_

#include <vector>
#include <thread>

#include <utilities.h>
#include "Task.h"

namespace ev3way {

	class EV3Way : ev3lib::noncopyable {
	private:
		std::vector<std::thread> m_threads;

		void stop();

		void threadFunc(size_t taskPos);

	public:
		EV3Way();

		void addTask(Task&& task);

		void start();

		void wait();
	};

}



#endif /* EV3WAY_H_ */
