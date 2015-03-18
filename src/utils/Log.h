/*
 * Log.h
 *
 *  Created on: 18 марта 2015 г.
 *      Author: Max
 */

#ifndef LOG_H_
#define LOG_H_

#include <vector>
#include "../Time.h"

namespace ev3way {

class Log {
public:
	struct State {
		clock_type::time_point time;
		float gyro_value;
		float accel_angle;
		float motor_pos;

		float psi;
		float psidot;
		float theta;
		float thetadot;

		float theta_ref;
		float thetadot_ref;

		float voltage;
		float volume;
		float err_theta;
	};

private:
	static std::vector<State> states;
public:

	static void add(const State& state) {
		states.push_back(state);
	}

	static size_t size() {
		return states.size();
	}

	static void print();

};

} /* namespace ev3way */

#endif /* LOG_H_ */
