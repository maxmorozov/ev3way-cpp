/*
 * LowPassFilterEstimator.cpp
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#include "../Constants.h"
#include "LowPassFilterEstimator.h"

namespace ev3way {
/**
 * Returns angular velocity
 *
 * @return angular velocity in degree/sec
 */
float LowPassFilterEstimator::getAngularVelocity() const {
    return angularVelocity;
}

/**
 * Returns body angle
 *
 * @return body angle in degree
 */
float LowPassFilterEstimator::getAngle() const {
    return angle;
}

/**
 * Update the estimator's state
 *
 * @param gyroValue angular velocity in degree/sec
 * @param interval  execution interval in seconds
 */
void LowPassFilterEstimator::updateState(float gyroValue, float interval) {
    //Use low-pass filter to find average angular velocity
    gyroOffset = gyroOffset * Constants::GYRO_COMPENSATION_FILTER + (1 - Constants::GYRO_COMPENSATION_FILTER) * gyroValue;
    //Use low-pass filter to reduce gyro noise
    angularVelocity = Constants::GYRO_FILTER * angularVelocity + (1 - Constants::GYRO_FILTER) * (gyroValue - gyroOffset);

    angle = nextAngle;
    nextAngle += (interval * angularVelocity);
}

/**
 * Updates the initial gyroscope offset
 *
 * @param gyroOffset initial gyro offset in degrees
 */
void LowPassFilterEstimator::init(float gyroOffset) {
    this->gyroOffset = gyroOffset;
}

float LowPassFilterEstimator::getGyroOffset() const{
    return gyroOffset;
}


}


