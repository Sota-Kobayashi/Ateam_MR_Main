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

constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS1	= {-0.25f, -0.007f, -0.0068f, 5000, 8192};
constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS2	= {-0.25f, -0.007f, -0.0068f, 5000, 8192};
constexpr MotorDriver::SendParamsType 	MOTOR_PARAMS3	= {-0.25f, -0.007f, -0.0068f, 5000, 8192};
constexpr MotorDriver::SendParamsType	MOTOR_PARAMS4	= {-0.25f, -0.007f, -0.0068f, 5000, 8192};
constexpr float 						WHEEL_DIA		= 101.6f;
constexpr bool							WHEEL_DIR		= false;
constexpr float							THETA_OFFSET	= 0.25f * M_PI;


constexpr int32_t 			AUTO_RUNNING_ERROR	= 150;
constexpr float 			ROLLING_SPEED		= 500.0f;
constexpr pid_gain<float> 	ROLLING_GAIN		= {-22.0f, 0.00f, -1.0f};
constexpr pid_gain<float> 	RUNNING_GAIN		= {47.75f, 1.6f, 0.16f};

constexpr SbdbtDualShock::Pad		RUN_PAD					= SbdbtDualShock::Pad::LEFT;
constexpr SbdbtDualShock::Buttons	RUN_RIGHTANGLE_MODE		= SbdbtDualShock::Buttons::L2;
constexpr SbdbtDualShock::Buttons	FAST_MODE				= SbdbtDualShock::Buttons::R2;
constexpr SbdbtDualShock::Buttons	TURN_RIGHT				= SbdbtDualShock::Buttons::R1;
constexpr SbdbtDualShock::Buttons	TURN_LEFT				= SbdbtDualShock::Buttons::L1;
constexpr SbdbtDualShock::Pad		TURN_CONTROL_PAD		= SbdbtDualShock::Pad::RIGHT;
constexpr SbdbtDualShock::Buttons	GET_BATHTOWEL			= SbdbtDualShock::Buttons::RIGHT;
constexpr SbdbtDualShock::Buttons	GET_T_SHIRT				= SbdbtDualShock::Buttons::LEFT;
constexpr SbdbtDualShock::Buttons	GET_SHEETS				= SbdbtDualShock::Buttons::UP;
constexpr SbdbtDualShock::Buttons	OUTPUT_BASKET			= SbdbtDualShock::Buttons::CIRCLE;
constexpr SbdbtDualShock::Buttons	HANG_T_SHIRT_1			= SbdbtDualShock::Buttons::CIRCLE;
constexpr SbdbtDualShock::Buttons	HANG_T_SHIRT_2			= SbdbtDualShock::Buttons::SQUARE;
constexpr SbdbtDualShock::Buttons	HANG_BATHTOWEL			= SbdbtDualShock::Buttons::CIRCLE;
constexpr SbdbtDualShock::Buttons	INCREMENT_MODE			= SbdbtDualShock::Buttons::START;
constexpr SbdbtDualShock::Buttons 	DECREMENT_MODE			= SbdbtDualShock::Buttons::SELECT;

constexpr uint32_t	TIMEOUT					= 100;
constexpr float		MAX_VEL_MANUAL			= 1000.0f;
constexpr float		MAX_VEL_MANUAL_FAST		= 2500.0f;
constexpr float		MAX_ROLLING_VEL_MANUAL	= 100.0f;
constexpr float		MAX_ROLLING_VEL_AUTO	= 200.0f;

constexpr OmniWheel4s::OmniWheel4s_ParamStruct OMNI_PARAMS_STRUCT =
{
	.wheel1_param	= MOTOR_PARAMS1,
	.wheel2_param	= MOTOR_PARAMS2,
	.wheel3_param	= MOTOR_PARAMS3,
	.wheel4_param	= MOTOR_PARAMS4,
	.wheel_dia		= WHEEL_DIA,
	.direction		= WHEEL_DIR,
	.angle_offset	= THETA_OFFSET
};

constexpr AutoRunning::AutoRunningParamsStruct AUTO_RUNNING_STRUCT =
{
	.omni_params		= OMNI_PARAMS_STRUCT,
	.acceptable_error	= AUTO_RUNNING_ERROR,
	.rolling_speed		= ROLLING_SPEED,
	.rolling_gain		= ROLLING_GAIN,
	.running_gain		= RUNNING_GAIN
};

constexpr Sequence::ButtonsFunctionStruct BUTTONS_FUNCTION_STRUCT =
{
	.run_pad				= RUN_PAD,
	.run_rightangle_mode	= RUN_RIGHTANGLE_MODE,
	.fast_mode				= FAST_MODE,
	.turn_right				= TURN_RIGHT,
	.turn_left				= TURN_LEFT,
	.turn_control_pad		= TURN_CONTROL_PAD,
	.get_bathtowel			= GET_BATHTOWEL,
	.get_T_shirt			= GET_T_SHIRT,
	.get_sheets				= GET_SHEETS,
	.output_basket			= OUTPUT_BASKET,
	.hang_T_shirt_1			= HANG_T_SHIRT_1,
	.hang_T_shirt_2			= HANG_T_SHIRT_2,
	.hang_bathtowel			= HANG_BATHTOWEL,
	.increment_mode			= INCREMENT_MODE,
	.decrement_mode			= DECREMENT_MODE
};

constexpr Sequence::ParamatersStruct SEQUENCE_PARAMS_STRUCT =
{
	.autorunning_params		= AUTO_RUNNING_STRUCT,
	.buttons_function		= BUTTONS_FUNCTION_STRUCT,
	.sbdbt_timeout_time_ms	= TIMEOUT,
	.run_max_vel			= MAX_VEL_MANUAL,
	.run_max_vel_fast		= MAX_VEL_MANUAL_FAST,
	.rolling_vel_manual		= MAX_ROLLING_VEL_MANUAL,
	.rolling_vel_auto		= MAX_ROLLING_VEL_AUTO
};



#endif /* PARAMS_HPP_ */
