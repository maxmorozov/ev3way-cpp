/*
 * LowPassFilterEstimator.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef LOWPASSFILTERESTIMATOR_H_
#define LOWPASSFILTERESTIMATOR_H_

#include "../StateVariablesEstimator.h"

namespace ev3way {

class LowPassFilterEstimator : public StateVariablesEstimator {
private:
    float gyroOffset = 0;
    float angularVelocity = 0;
    float angle = 0;
    float nextAngle = 0;

public:
    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    float getAngularVelocity() const override;

    /**
     * Returns body angle
     *
     * @return body angle in degree
     */
    float getAngle() const override;

    /**
     * Update the estimator's state
     *
     * @param gyroValue angular velocity in degree/sec
     * @param interval  execution interval in seconds
     */
    void updateState(float gyroValue, float interval) override;
    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    void init(float gyroOffset) override;

    float getGyroOffset() const override;

};

} /* namespace ev3way */

#endif /* LOWPASSFILTERESTIMATOR_H_ */
