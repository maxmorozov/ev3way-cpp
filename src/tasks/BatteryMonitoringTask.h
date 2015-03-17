/*
 * BatteryMonitoringTask.h
 *
 *  Created on: 28 февр. 2015 г.
 *      Author: Max
 */

#ifndef BATTERYMONITORINGTASK_H_
#define BATTERYMONITORINGTASK_H_

#include "../utils/SharedState.h"

namespace ev3way {

class BatteryMonitoringTask {
private:
    SharedState* m_state;

public:
	BatteryMonitoringTask(SharedState* state);

	void operator()();
};

} /* namespace ev3way */

#endif /* BATTERYMONITORINGTASK_H_ */
