/*
 * Constants.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <math.h>

namespace ev3way {

class Constants {
public:
    // Low Path Filter Coefficients
    static constexpr float BATTERY_FILTER = 0.8f;       // average battery value
    static constexpr float GYRO_CALIBRATION_FILTER = 0.8f;	    // calibrate gyro offset
    static constexpr float GYRO_COMPENSATION_FILTER = 0.999f;	// compensate gyro drift

    static const int CONTROLLER_TIME = 8; //8 mc
    /**
     * Number of milliseconds for calibrating gyro
     */
    static const int GYRO_CALIBRATION_PERIOD = 1000;

    /**
     * Number of offset samples to average when calculating gyro offset.
     */
    static const int GYRO_OFFSET_SAMPLES = 100;

    /**
     * Time to balance the robot before starting the autonomous movement
     */
    static const int INITIAL_BALANCING_PERIOD = 5000;

    /* minimum distance in cm for obstacle avoidance */
    static const int MIN_DISTANCE = 80;

    /**
     * Converts radians to degrees
     */
    static constexpr float RAD_TO_DEGREE = (float)(180 / M_PI);
    static constexpr float DEG2RAD = (float) (M_PI/180);
};

} /* namespace ev3way */

#endif /* CONSTANTS_H_ */
