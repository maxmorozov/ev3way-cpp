/*
 * BalanceController.h
 *
 *  Created on: 27 февр. 2015 г.
 *      Author: Max
 */

#ifndef BALANCECONTROLLER_H_
#define BALANCECONTROLLER_H_

#include "../Constants.h"
#include "EV3Wheels.h"
#include "MotorbikeWheels.h"
#include "../Time.h"

namespace ev3way {

class BalanceController : public MotorbikeWheels {
private:
    /**
     * If robot power is saturated (over +/- 100) for over this time limit then
     * robot must have fallen.  In milliseconds.
     */
    static std::chrono::milliseconds TIME_FALL_LIMIT;

    static constexpr float CMD_MAX = 100.0f;
    static constexpr float POWER_MAX = 100.0f;

    static constexpr float EXEC_PERIOD = Constants::CONTROLLER_TIME / 1000.0f;
    static constexpr float EXEC_FREQUENCY = 1 / EXEC_PERIOD;

    static constexpr float A_D = 0.9f;//0.8F;       /* low pass filter gain for motors average count */
    static constexpr float A_R = 0.996F;     /* low pass filter gain for motors target count */

    static constexpr float K_PHIDOT = 25.0F;         /* turn target speed gain */
    static constexpr float K_SYNC = 0.35F;           /* wheel synchronization gain */

    static constexpr float BATTERY_GAIN = 0.001089F; /* battery voltage gain for motor PWM outputs */
    static constexpr float BATTERY_OFFSET = 0.625F;  /* battery voltage offset for motor PWM outputs */

    //state variables on the previous step
	float prior_err_theta = 0;
	float prior_theta_lpf = 0;
	float prior_theta_ref = 0;
	float prior_thetadot_cmd_lpf = 0; //wheels angular velocity, filtered by Low Path Filter to suppress rapid input change.

    //Wheels synchronization part
     bool prior_flag_turn = false;
     int theta_offset = 0;

    //these variables are used to detect falling
     bool firstStep = true;
     clock_type::time_point lastGoodRegulationTime;

     float saturate(float value, float min, float max);

public:
     /**
      * Performs the regulation cycle. Execution time is 0.583 ms
      *
      * @param cmd_forward     speed of the forward movement. -100(backward max.) to 100(forward max.)
      * @param cmd_turn        speed of the turning. -100(turn left max.) to 100(turn right max.)
      * @param psidot          the body angular velocity
      * @param psi             body pitch
      * @param left_motor_pos  left motor rotor position in degrees
      * @param right_motor_pos right motor rotor position in degrees
      * @param battery_voltage the battery voltage in milli-volts (mV)
      * @return encoded power for left and right motors. Low byte - left motor, high byte - right motor
      */
     short control(int cmd_forward, int cmd_turn, float psidot, float psi, int left_motor_pos, int right_motor_pos, float battery_voltage);

     /**
      * Check if robot has fallen by detecting that motor power is being limited
      * for an extended amount of time.
      *
      * @return true if the motor power is not saturated
      */
     bool isOk();
};

} /* namespace ev3way */

#endif /* BALANCECONTROLLER_H_ */
