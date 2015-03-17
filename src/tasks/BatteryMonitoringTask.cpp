/*
 * BatteryMonitoringTask.cpp
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#include "BatteryMonitoringTask.h"

namespace ev3way {

BatteryMonitoringTask::BatteryMonitoringTask(SharedState* state)
	: m_state(state)
{

}

void BatteryMonitoringTask::operator()() {
	m_state->updateBatteryVoltage();
}

} /* namespace ev3way */
