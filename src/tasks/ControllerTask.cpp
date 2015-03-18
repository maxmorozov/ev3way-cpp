/*
 * ControllerTask.cpp
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */
#undef max
#undef min

#include <iostream>
#include <algorithm>
#include "ControllerTask.h"

using namespace std;

namespace ev3way {

void ControllerTask::operator()() {
    switch (currentState) {
        case State::Init:
            // Ensure that the motor controller is active since this affects the gyro values.
            leftMotor.stop();
            rightMotor.stop();

            leftMotor.resetTachoCount();
            rightMotor.resetTachoCount();
            gyroMax = -1000;
            gyroMin = 1000;
            gyroOffset = 0;
            avgCount = 0;

            currentState = State::Calibrating;
            displayCalibrating();
            break;

        case State::Calibrating: {
            //calibrate using low pass filter
            //gyroOffset = gyroOffset * Constants.GYRO_CALIBRATION_FILTER + (1 - Constants.GYRO_CALIBRATION_FILTER) * gyro.readValue();

            //calibrate using average value of gyro
            float gyroValue = readValue();

            gyroOffset += gyroValue;
            ++avgCount;
            gyroMax = max(gyroMax, gyroValue);
            gyroMin = min(gyroMin, gyroValue);

            if (avgCount == Constants::GYRO_OFFSET_SAMPLES) {
            	cout<<"Delta: "<<(gyroMax - gyroMin)<<endl;
                if ((gyroMax - gyroMin) > 4) {
                    currentState = State::Init;
                } else {
                    gyroOffset /= avgCount;
                    estimator->init(gyroOffset);

                    currentState = State::Prepare;
                    displayBalanceWarning();
                }
            }
            break;
        }

        case State::Prepare:
            // Play warning beep sequence each second before balance starts
            if ((countDown % BEEP_INTERVAL) == 0) {
            	cout<<(countDown / BEEP_INTERVAL)<<endl;
                //LCD.drawInt(countDown / BEEP_INTERVAL, 5, 4);
                //Sound.playTone(440, 100);
            }
            if (--countDown == 0) {
                //LCD.clear();
                currentState = State::Control;
                //shared.setBalancing(true);
            }
            break;

        case State::Control: {
            short navigation = 0;//navigator.getControl();
            char cmd_forward = (char) (navigation & 0xff);
            char cmd_turn = (char) ((navigation >> 8) & 0xff);

            float gyroValue = readValue();
            estimator->updateState(gyroValue, EXEC_PERIOD);

            short result = controller.control(
                    cmd_forward,
                    cmd_turn,
                    estimator->getAngularVelocity(),
                    estimator->getAngle(),
                    leftMotor.getTachoCount(),
                    rightMotor.getTachoCount(),
                    shared->getBatteryVoltage(),
                    gyroValue - gyroOffset,
                    0);

            //Check if the robot has fallen
            if (!controller.isOk()) {
                //Sound.beepSequenceUp();
                //LCD.drawString("Oops... I fell", 0, 4);
            	cout<<"Oops... I fell"<<endl;

                leftMotor.flt();
                rightMotor.flt();

                stopper();

                result = 0;
            }
            char left_motor_power = (char) (result & 0xFF);
            char right_motor_power = (char) ((result >> 8) & 0xFF);

            controlMotor(leftMotor, left_motor_power);
            controlMotor(rightMotor, right_motor_power);
        }
        break;

        default:
            leftMotor.stop();
            rightMotor.stop();

    }
}

float ControllerTask::readValue() {
	return gyro.getData();
}

void ControllerTask::displayCalibrating() {
    cout<<"leJOS EV3Way"<<endl
    	<<"Lay robot down flat to get gyro offset."<<endl;
}

/**
 * Warn user the NXJWay is about to start balancing.
 */
void ControllerTask::displayBalanceWarning() {
    cout<<"Balance in"<<endl;
}

void ControllerTask::controlMotor(ev3lib::hardware::UnregulatedMotor& motor, int power) {
    motor.setPower(abs(power));
    if (power > 0)
        motor.forward();
    else
        motor.backward();
}

} /* namespace ev3way */
