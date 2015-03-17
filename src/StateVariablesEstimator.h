/*
 * StateVariablesEstimator.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef STATEVARIABLESESTIMATOR_H_
#define STATEVARIABLESESTIMATOR_H_

#include <utilities.h>

namespace ev3way {

struct StateVariablesEstimator : public ev3lib::destructible  {
    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    virtual float getAngularVelocity() const = 0;

    /**
     * Returns body angle
     *
     * @return body angle in degree
     */
    virtual float getAngle() const = 0;

    /**
     * Update the estimator's state
     *
     * @param gyroValue angular velocity in degree/sec
     * @param interval execution interval in seconds
     */
    virtual void updateState(float gyroValue, float interval) = 0;

    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    virtual void init(float gyroOffset) = 0;

    virtual float getGyroOffset() const = 0;
};

} /* namespace ev3way */

#endif /* STATEVARIABLESESTIMATOR_H_ */
