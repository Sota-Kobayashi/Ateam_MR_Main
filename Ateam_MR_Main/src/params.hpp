/*
 * params.hpp
 *
 *  Created on: 2019/10/10
 *      Author: SotaKobayashi
 */

#ifndef PARAMS_HPP_
#define PARAMS_HPP_

#include "omni_wheel.hpp"
#include "auto_running.hpp"
#include "sequence.hpp"

constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS1	= {-0.017f, -0.005f, -0.0007f, 5000, 8192};
constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS2	= {-0.017f, -0.005f, -0.0007f, 5000, 8192};
constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS3	= {-0.017f, -0.005f, -0.0007f, 5000, 8192};
constexpr MotorDriver::SendParamsType	MOTOR_PARAMS4	= {-0.017f, -0.005f, -0.0007f, 5000, 8192};
constexpr float 						WHEEL_DIA		= 101.6f;
constexpr bool							WHEEL_DIR		= false;
constexpr float							THETA_OFFSET	= 0.75f * M_PI;


constexpr int32_t 			AUTO_RUNNING_ERROR	= 150;
constexpr float 			ROLLING_SPEED		= 100.0f;
constexpr pid_gain<float> 	ROLLING_GAIN		= {150.0f, 0.00f, 2.0f};
constexpr pid_gain<float> 	RUNNING_GAIN		= {300.0f, 10.0f, 1.0f};

constexpr SbdbtDualShock::Pad		RUN_PAD					= SbdbtDualShock::Pad::LEFT;
constexpr SbdbtDualShock::Buttons	RUN_RIGHTANGLE_MODE		= SbdbtDualShock::Buttons::L2;
constexpr SbdbtDualShock::Buttons	FAST_MODE				= SbdbtDualShock::Buttons::R2;
constexpr SbdbtDualShock::Buttons	TURN_RIGHT				= SbdbtDualShock::Buttons::R1;
constexpr SbdbtDualShock::Buttons	TURN_LEFT				= SbdbtDualShock::Buttons::L1;
constexpr SbdbtDualShock::Buttons	TURN_RIGHTANGLE_MODE	= SbdbtDualShock::Buttons::CROSS;

constexpr uint32_t	TIMEOUT					= 100;
constexpr float		MAX_VEL_MANUAL			= 200.0f;
constexpr float		MAX_VEL_MANUAL_FAST		= 500.0f;
constexpr float		MAX_ROLLING_VEL_MANUAL	= 100.0f;

constexpr OmniWheel4s::OmniWheel4s_ParamStruct OMNI_PARAMS_STRUCT =
{
	MOTOR_PARAMS1,
	MOTOR_PARAMS2,
	MOTOR_PARAMS3,
	MOTOR_PARAMS4,
	WHEEL_DIA,
	WHEEL_DIR,
	THETA_OFFSET
};

constexpr AutoRunning::AutoRunningParamsStruct AUTO_RUNNING_STRUCT =
{
	OMNI_PARAMS_STRUCT,
	AUTO_RUNNING_ERROR,
	ROLLING_SPEED,
	ROLLING_GAIN,
	RUNNING_GAIN
};

constexpr Sequence::ButtonsFunctionStruct BUTTONS_FUNCTION_STRUCT =
{
	RUN_PAD,
	RUN_RIGHTANGLE_MODE,
	FAST_MODE,
	TURN_RIGHT,
	TURN_LEFT,
	TURN_RIGHTANGLE_MODE
};

constexpr Sequence::ParamatersStruct SEQUENCE_PARAMS_STRUCT =
{
	AUTO_RUNNING_STRUCT,
	BUTTONS_FUNCTION_STRUCT,
	TIMEOUT,
	MAX_VEL_MANUAL,
	MAX_VEL_MANUAL_FAST,
	MAX_ROLLING_VEL_MANUAL
};



#endif /* PARAMS_HPP_ */
