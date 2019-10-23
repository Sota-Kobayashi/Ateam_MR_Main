/*
 * sequence.hpp
 *
 *  Created on: 2019/10/03
 *      Author: SotaKobayashi
 */

#ifndef SEQUENCE_HPP_
#define SEQUENCE_HPP_

#include <cmath>
#include "sbdbt.hpp"
#include "auto_running.hpp"
#include "led.hpp"

class Sequence
{
public:
	struct ButtonsFunctionStruct
	{
		SbdbtDualShock::Pad run_pad;
		SbdbtDualShock::Buttons run_rightangle_mode;
		SbdbtDualShock::Buttons fast_mode;
		SbdbtDualShock::Buttons turn_right;
		SbdbtDualShock::Buttons turn_left;
		SbdbtDualShock::Pad turn_control_pad;

		SbdbtDualShock::Buttons get_bathtowel;
		SbdbtDualShock::Buttons get_T_shirt;
		SbdbtDualShock::Buttons get_sheets;
		SbdbtDualShock::Buttons output_basket;
		SbdbtDualShock::Buttons hang_T_shirt_1;
		SbdbtDualShock::Buttons hang_T_shirt_2;
		SbdbtDualShock::Buttons	hang_bathtowel;
		SbdbtDualShock::Buttons increment_mode;
		SbdbtDualShock::Buttons decrement_mode;
	};
	struct ParamatersStruct
	{
		AutoRunning::AutoRunningParamsStruct autorunning_params;
		ButtonsFunctionStruct buttons_function;
		uint32_t sbdbt_timeout_time_ms;
		float run_max_vel;
		float run_max_vel_fast;
		float rolling_vel_manual;
		float rolling_vel_auto;
	};

	Sequence() = delete;
	Sequence(const bool& emergency_state, const ParamatersStruct& params, UART& gyro_uart_interface, UART& sbdbt_uart_interface, ControlAreaNetwork& can_interface) :
		params_(params),
		auto_running_(emergency_state, params.autorunning_params, gyro_uart_interface, can_interface),
		dualshock_(params.sbdbt_timeout_time_ms, sbdbt_uart_interface),
		systick_([&](){ manualRunningControl_(); mechaControl(); }),
		can_interface_(can_interface)
	{

	}
private:
	const ParamatersStruct params_;
	AutoRunning auto_running_;
	SbdbtDualShock dualshock_;
	LED<LED_Color::Green> led_g_;
	SysTick_Interrupt systick_;
	ControlAreaNetwork can_interface_;

	enum class Mode
	{
		STANDBY,
		RECOVERRY,
		HANG_T_SHIRT,
		HANG_BATH_TOWEL
	};

	enum Telegram : uint8_t
	{
		GET_BATHTOWEL		= 0x01,
		GET_T_SHIRT			= 0x02,
		GET_SHEETS			= 0x03,
		OUTPUT_BASKET		= 0x04,
		HANG_T_SHIRT_1		= 0x05,
		HANG_T_SHIRT_2		= 0x06,
		HANG_BATHTOWEL		= 0x07,
		GO_ZEROPOINT		= 0x08,
		GO_T_SHIRT_POINT	= 0x09,
		GO_BATHTOWEL_POINT	= 0x10

	};

	int16_t target_anguler_pos_deg_ = 0;

	static constexpr uint8_t sequence_getting_land = 0x01;

	void controlGettingLandryPos_();
	void manualRunningControl_();
	void mechaControl();
};

#endif /* SEQUENCE_HPP_ */
