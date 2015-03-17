/*
 * MotorbikeWheels.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef MOTORBIKEWHEELS_H_
#define MOTORBIKEWHEELS_H_

#include "../Constants.h"

namespace ev3way {

class MotorbikeWheels {
protected:
	static constexpr float K_F1 = -0.824844186088244F * Constants::DEG2RAD;
	static constexpr float K_F2 = -41.4074441149571F * Constants::DEG2RAD;
	static constexpr float K_F3 = -1.32276811679559F * Constants::DEG2RAD;
	static constexpr float K_F4 = -4.40904857993352F * Constants::DEG2RAD;

	static constexpr float K_I = -0.417915074452005F * Constants::DEG2RAD; // servo control integral gain

	static constexpr float K_THETADOT = 6.31578947368421F / Constants::DEG2RAD;   // forward target speed gain 0.2 m/s
};

} /* namespace ev3way */

#endif /* MOTORBIKEWHEELS_H_ */
