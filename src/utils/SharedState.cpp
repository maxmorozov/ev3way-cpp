/*
 * SharedState.cpp
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#include "SharedState.h"
#include "../Constants.h"

namespace ev3way {

SharedState::SharedState(ev3lib::hardware::Battery battery)
	: m_battery(battery)
{
	m_batteryVoltage = m_battery.getVoltageMilliVolt();
}


float SharedState::getBatteryVoltage() const {
    return m_batteryVoltage;
}

/**
 * Updates the current battery voltage using Low-Pass filter to reduce noise
 */
void SharedState::updateBatteryVoltage() {
    m_batteryVoltage = m_batteryVoltage * Constants::BATTERY_FILTER + (1 - Constants::BATTERY_FILTER) * m_battery.getVoltageMilliVolt();
}

const clock_type::time_point SharedState::getStartTime() const {
    if (m_isBalancing)
        return m_startTime;
    else
        return clock_type::now();
}

bool SharedState::isBalancing() const {
    return m_isBalancing;
}

void SharedState::setBalancing(bool balancing) {
    m_startTime = clock_type::now();
    m_isBalancing = balancing;
}

float SharedState::getBodyAngle() const {
    return m_bodyAngle;
}

void SharedState::setBodyAngle(float bodyAngle) {
    m_bodyAngle = bodyAngle;
}

} /* namespace ev3way */
