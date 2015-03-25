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
	static constexpr float K_F1 = -0.825481360095411F * Constants::DEG2RAD;
	static constexpr float K_F2 = -40.3738849807044F * Constants::DEG2RAD;
	static constexpr float K_F3 = -1.32450621391743F * Constants::DEG2RAD;
	static constexpr float K_F4 = -4.22675237197456F * Constants::DEG2RAD;

	static constexpr float K_I = -0.417669314488814F * Constants::DEG2RAD; // servo control integral gain

	static constexpr float K_THETADOT = 6.31578947368421F / Constants::DEG2RAD;   // forward target speed gain 0.2 m/s
};

} /* namespace ev3way */

#endif /* MOTORBIKEWHEELS_H_ */
