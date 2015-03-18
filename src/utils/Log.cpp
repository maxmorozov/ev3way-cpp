/*
 * Log.cpp
 *
 *  Created on: 18 марта 2015 г.
 *      Author: Max
 */

#include <iostream>
#include <fstream>
#include "Log.h"

using namespace std;

namespace ev3way {

std::vector<Log::State> Log::states;

void Log::print() {
	ofstream os("ev3way.log");
	for(const auto& st: states)
	{
		os<<st.time.time_since_epoch().count()<<","<<st.gyro_value<<","<<st.accel_angle<<","<<st.motor_pos<<","<<
            st.psi<<","<<st.psidot<<","<<st.theta<<","<<st.thetadot<<","<<st.theta_ref<<","<<
            st.thetadot_ref<<","<<st.voltage<<","<<st.volume<<","<<st.err_theta<<endl;
    }
}


} /* namespace ev3way */
