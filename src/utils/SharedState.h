/*
 * SharedState.h
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#ifndef SHAREDSTATE_H_
#define SHAREDSTATE_H_

#include <hardware/Battery.h>
#include "../Time.h"

namespace ev3way {

class SharedState {
private:
	ev3lib::hardware::Battery m_battery;

    volatile float m_batteryVoltage;

    /**
     * Time when balancing was started
     */
    clock_type::time_point m_startTime;

    /**
     * Indicates that the robot is balancing now
     */
    volatile bool m_isBalancing = false;

    /**
     * The robot's body angle measured by the accelerometer task.
     * Accelerometer sensor is slow (requires about 10 ms for reading) and it's polled by a specialized task
     */
    volatile float m_bodyAngle = 0;
public:
	SharedState(ev3lib::hardware::Battery battery);

    float getBatteryVoltage() const;

    /**
     * Updates the current battery voltage using Low-Pass filter to reduce noise
     */
    void updateBatteryVoltage();

    const clock_type::time_point getStartTime() const;

    bool isBalancing() const;

    void setBalancing(bool balancing);

    float getBodyAngle() const;

    void setBodyAngle(float bodyAngle);
};

} /* namespace ev3way */

#endif /* SHAREDSTATE_H_ */
