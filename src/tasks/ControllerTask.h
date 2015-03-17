/*
 * ControllerTask.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef CONTROLLERTASK_H_
#define CONTROLLERTASK_H_

#include <functional>
#include <hardware/UnregulatedMotor.h>
#include <hardware/HiTechnicGyro.h>

#include "../StateVariablesEstimator.h"
#include "../controller/BalanceController.h"
#include "../utils/SharedState.h"

namespace ev3way {

class ControllerTask {
private:
	enum class State {
		Init,
		Calibrating,
		Prepare,
		Control
	};

    State currentState = State::Init;

    SharedState* shared;
    //final Navigator navigator;
    BalanceController controller;

    ev3lib::hardware::HiTechnicGyro gyro;
    ev3lib::hardware::UnregulatedMotor leftMotor;
    ev3lib::hardware::UnregulatedMotor rightMotor;

    std::unique_ptr<StateVariablesEstimator> estimator;

    //Gyro offset
    float gyroOffset = 0;
    int avgCount = 0;
    float gyroMax, gyroMin;

    //Prepare to balance
    //1 sec = 250 periods
    static const int BEEP_INTERVAL = 1000 / Constants::CONTROLLER_TIME;
    int countDown = BEEP_INTERVAL * 6; //6 seconds

    static constexpr float EXEC_PERIOD = Constants::CONTROLLER_TIME / 1000.0f;

    std::function<void()> stopper;

    float readValue();

    static void displayCalibrating();
    /**
     * Warn user the NXJWay is about to start balancing.
     */
    static void displayBalanceWarning();

    void controlMotor(ev3lib::hardware::UnregulatedMotor& motor, int power);

public:
    template <typename T>
	ControllerTask(ev3lib::hardware::HiTechnicGyro&& gyroSensor, const ev3lib::hardware::UnregulatedMotor& left, const ev3lib::hardware::UnregulatedMotor& right,
			SharedState* sharedState,
			std::unique_ptr<StateVariablesEstimator>&& stateEstimator, T stop)
		:  shared(sharedState), gyro(std::move(gyroSensor)), leftMotor(left), rightMotor(right), estimator(std::move(stateEstimator)),  stopper(stop)
	{
    	gyroMax = gyroMin = 0;
    }

    void operator()();
};

} /* namespace ev3way */

#endif /* CONTROLLERTASK_H_ */
