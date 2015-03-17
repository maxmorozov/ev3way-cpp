/*
 * EV3Wheels.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef EV3WHEELS_H_
#define EV3WHEELS_H_

#include "../Constants.h"

namespace ev3way {

class EV3Wheels {
protected:
	static constexpr float K_F1 = -0.770037357557574F * Constants::DEG2RAD;
	static constexpr float K_F2 = -84.1899879423200F * Constants::DEG2RAD;
	static constexpr float K_F3 = -1.22027762814827F * Constants::DEG2RAD;
	static constexpr float K_F4 = -9.28721276122188F * Constants::DEG2RAD;

	static constexpr float K_I = -0.416689176972853F * Constants::DEG2RAD; // servo control integral gain

	static constexpr float K_THETADOT = 9.30232558139535F / Constants::DEG2RAD;   // forward target speed gain 0.2 m/s
};

} /* namespace ev3way */

#endif /* EV3WHEELS_H_ */
