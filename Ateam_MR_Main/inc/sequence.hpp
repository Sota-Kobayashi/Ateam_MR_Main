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
		SbdbtDualShock::Buttons turn_rightangle_mode;
	};
	struct ParamatersStruct
	{
		AutoRunning::AutoRunningParamsStruct autorunning_params;
		ButtonsFunctionStruct buttons_function;
		uint32_t sbdbt_timeout_time_ms;
		float run_max_vel;
		float run_max_vel_fast;
		float rolling_vel_manual;
	};

	Sequence() = delete;
	Sequence(const ParamatersStruct& params, UART& gyro_uart_interface, UART& sbdbt_uart_interface, ControlAreaNetwork& can_interface) :
		params_(params),
		auto_running_(params.autorunning_params, gyro_uart_interface, can_interface),
		dualshock_(params.sbdbt_timeout_time_ms, sbdbt_uart_interface),
		systick_([&](){ manualRunningControl_(); })
	{

	}
private:
	const ParamatersStruct params_;
	AutoRunning auto_running_;
	SbdbtDualShock dualshock_;
	LED<LED_Color::Orange> led_g_;
	SysTick_Interrupt systick_;

	int16_t target_anguler_pos_deg_ = 0;

	static constexpr uint8_t sequence_getting_land = 0x01;

	void controlGettingLandryPos_();
	void manualRunningControl_();
};

#endif /* SEQUENCE_HPP_ */
