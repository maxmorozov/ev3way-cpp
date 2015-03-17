//============================================================================
// Name        : MeasureCurve.cpp
// Author      : Max
// Version     :
// Copyright   : 
// Description : Hello World in C++
//============================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <iterator>
#include <algorithm>

#include <boost/exception/all.hpp>

#include <hardware/Brick.h>

#include "Time.h"
#include "utils/SharedState.h"
#include "estimators/LowPassFilterEstimator.h"
#include "tasks/ControllerTask.h"
#include "tasks/BatteryMonitoringTask.h"

#include "Constants.h"

#include "EV3Way.h"

#include <pthread.h>
#include <sched.h>

using namespace std;
using namespace ev3lib::hardware;

namespace ev3way {

	EV3Way::EV3Way() {

	}

	void EV3Way::addTask(Task&& task) {
		m_threads.push_back(thread(task));
	}

	void EV3Way::addRealTimeTask(Task&& task) {
		m_threads.push_back(thread(task));

		sched_param    param;
		param.sched_priority = sched_get_priority_min(SCHED_RR);
		cout<<"Priority: "<<param.sched_priority<<endl;
		pthread_setschedparam(m_threads.back().native_handle(), SCHED_RR, &param);
	}

	void EV3Way::wait() {
		for_each(m_threads.begin(), m_threads.end(), [](thread& t) { t.join(); });
	}

	void start() {
		try {
			Brick brick;

			Battery battery = brick.getBattery();

			UnregulatedMotor rightMotor = brick.getMotor(Motors::A);
			UnregulatedMotor lefttMotor = brick.getMotor(Motors::D);

			HiTechnicGyro gyro = brick.getGyro(Sensors::S1);

	        SharedState state(battery);

	        Button escapeButton = brick.getButton(Buttons::Escape);

			volatile bool stop = false;

			auto stopper = [&]{ stop = true;};
			auto stopChecker = [&]()->bool{ return stop;};

			ControllerTask controller(move(gyro), lefttMotor, rightMotor, &state, unique_ptr<StateVariablesEstimator>(new LowPassFilterEstimator()), stopper);
			BatteryMonitoringTask batteryTask(&state);

			EV3Way robot;

			DeviationCalc calc;
		    clock_type::time_point time1 = clock_type::now();
		    int first_call = true;

			robot.addRealTimeTask(Task(chrono::milliseconds((int)Constants::CONTROLLER_TIME), [&]{
				if (first_call) {
					time1 = clock_type::now();
					first_call = false;
				} else {
				    clock_type::time_point time2 = clock_type::now();

				    chrono::nanoseconds interval = time2-time1;
			        float milliseconds = interval.count() / 1e6f;
			        calc.add(milliseconds);
			        time1 = time2;
				}

				controller();
			}, stopChecker));
			robot.addTask(Task(chrono::milliseconds(100), batteryTask, stopChecker));
			robot.addTask(Task(chrono::milliseconds(50), [&]{ if(escapeButton.isPressed()) stop = true; }, stopChecker));
			robot.addTask(Task(chrono::seconds(5), [&]{
				cout<<"Average: "<<calc.getAverage()<<", Deviation: "<<calc.getDeviation()<<endl;
			}, stopChecker));


			robot.wait();

		} catch (const boost::exception& e) {
			cout<<"Error: "<<boost::diagnostic_information(e)<<endl;
		}
	}

}
using namespace ev3way;

int main()
{
	start();

	return 0;
}

