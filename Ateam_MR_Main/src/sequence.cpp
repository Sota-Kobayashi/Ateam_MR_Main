/*
 * sequency.cpp
 *
 *  Created on: 2019/10/08
 *      Author: SotaKobayashi
 */

#include "sequence.hpp"
#include "tape_led.hpp"
#include "board_io.hpp"

void Sequence::controlGettingLandryPos_()
{
	static PID<float, float> rolling_pid(params_.autorunning_params.rolling_gain, auto_running_.dead_reckoning_.readPosTheta());
	static PID<float, float> running_pid(params_.autorunning_params.running_gain, auto_running_.dead_reckoning_.readPosX());

	/*PID step*/
	running_pid.setPidEnableState(true);
	running_pid.setLimitVal(running_path.at(sequence_getting_land).back().velocity_abs());
	running_pid.setTargetVal(running_path.at(sequence_getting_land).back().get_pos().posX());
	running_pid.update(auto_running_.dead_reckoning_.readPosX());

	rolling_pid.setPidEnableState(true);
	rolling_pid.setLimitVal(params_.autorunning_params.rolling_speed);
	rolling_pid.setTargetVal(running_path.at(sequence_getting_land).back().get_pos().posTheta());
	rolling_pid.update(auto_running_.dead_reckoning_.readPosTheta());

	vector2d<float> run_vector =
			{ running_pid.getControlVal(), params_.run_max_vel * dualshock_.readAnalogPad(params_.buttons_function.run_pad).getVector_rec().second};

	const float run_vector_por_local = run_vector.getVector_pol().second - (M_PI * 0.5) - auto_running_.dead_reckoning_.pos().posTheta();

	/*Drive wheel*/
	auto_running_.omni_wheel_.setRunningVector
		(run_vector.getVector_pol().first, run_vector_por_local, static_cast<int32_t>(rolling_pid.getControlVal()));
	return;
}

void Sequence::manualRunningControl_()
{
	//running control logic
	const float running_speed_ratio =
			dualshock_.isButtonPushed(params_.buttons_function.fast_mode) ? params_.run_max_vel_fast : params_.run_max_vel;

	const vector2d<float>& controler_analogpad_val = dualshock_.readAnalogPad(params_.buttons_function.run_pad);
	vector2d<float> run_vector = {};

	if(dualshock_.isButtonPushed(params_.buttons_function.run_rightangle_mode))
	{
		led_g_.setNewState(LED_State::ON);
		float x_vel = 0.0f;
		float y_vel = 0.0f;
		if(controler_analogpad_val.getVector_pol().second > (-0.750f * M_PI) && controler_analogpad_val.getVector_pol().second <= (-0.250f * M_PI))
		{
			y_vel = -1.0 * running_speed_ratio * std::abs(controler_analogpad_val.getVector_rec().second);
		}
		else if(controler_analogpad_val.getVector_pol().second > (-0.250f * M_PI) && controler_analogpad_val.getVector_pol().second <= (0.250f * M_PI))
		{
			x_vel = running_speed_ratio * std::abs(controler_analogpad_val.getVector_rec().first);
		}
		else if(controler_analogpad_val.getVector_pol().second > (0.250f * M_PI) && controler_analogpad_val.getVector_pol().second <= (0.750f * M_PI))
		{
			y_vel = running_speed_ratio * std::abs(controler_analogpad_val.getVector_rec().second);
		}
		else
		{
			x_vel = -1.0 * running_speed_ratio * std::abs(controler_analogpad_val.getVector_rec().first);
		}
		run_vector = make_vector2d<float>(x_vel, y_vel);
	}
	else
	{
		led_g_.setNewState(LED_State::OFF);
		const float x_vel = running_speed_ratio * controler_analogpad_val.getVector_rec().first;
		const float y_vel = running_speed_ratio * controler_analogpad_val.getVector_rec().second;
		run_vector = make_vector2d<float>(x_vel, y_vel);
	}

	const float run_vector_pol_local = run_vector.getVector_pol().second - auto_running_.dead_reckoning_.pos().posTheta();

	//rolling control logic
	/*
	const float rolling_vel = 	dualshock_.isButtonPushed(params_.buttons_function.turn_left) ? -params_.rolling_vel_manual	:
								dualshock_.isButtonPushed(params_.buttons_function.turn_right)? params_.rolling_vel_manual	: 0.0f;
	*/
	float rolling_vel = 0.0f;

	/*
	 * â…Ç™Ç†Ç¡ÇΩÇÁé¿ëïÇµÇƒÇŸÇµÇ¢ÅAê_Ç…Ç»ÇÍÇÈ
	 */
	static bool turn_right_was_pushed = false;
	static bool turn_left_was_pushed = false;
	static bool turn_pad_was_tilted = false;

	const int16_t now_pos_theta_deg = static_cast<int16_t>(auto_running_.dead_reckoning_.readPosTheta() * 180.0f / M_PI);

	constexpr int16_t rightangle_deg = 90;

	static PID<float, float> rolling_pid(params_.autorunning_params.rolling_gain, now_pos_theta_deg);

	if(!turn_right_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.turn_right))
	{
		if(target_anguler_pos_deg_ >= 0)
		{
			target_anguler_pos_deg_ -= (target_anguler_pos_deg_ % rightangle_deg) == 0 ? rightangle_deg : (target_anguler_pos_deg_ % rightangle_deg) ;
		}
		else
		{
			target_anguler_pos_deg_ -= rightangle_deg + (target_anguler_pos_deg_ % rightangle_deg);
		}
		//(target_anguler_pos_deg_ < -180)target_anguler_pos_deg_ += 180;
		rolling_pid.setPidEnableState(true);
	}
	else if(!turn_left_was_pushed &&  dualshock_.isButtonPushed(params_.buttons_function.turn_left))
	{
		if(target_anguler_pos_deg_ > 0)
		{
			target_anguler_pos_deg_ += rightangle_deg - (target_anguler_pos_deg_ % rightangle_deg);
		}
		else
		{
			target_anguler_pos_deg_ -= (target_anguler_pos_deg_ % rightangle_deg) == 0 ? -rightangle_deg : (target_anguler_pos_deg_ % rightangle_deg) ;
		}
		//while(target_anguler_pos_deg_ > 180)target_anguler_pos_deg_ -= 180;
		rolling_pid.setPidEnableState(true);
	}
	else if(std::abs(dualshock_.readAnalogPad(params_.buttons_function.turn_control_pad).getVector_rec().first) > 0.1f)
	{
		rolling_vel = dualshock_.readAnalogPad(params_.buttons_function.turn_control_pad).getVector_rec().first * params_.rolling_vel_manual;
		rolling_pid.setPidEnableState(false);
	}
	else
	{
		rolling_pid.setPidEnableState(true);
	}
	if(turn_pad_was_tilted)target_anguler_pos_deg_ = now_pos_theta_deg;
	rolling_pid.setTargetVal(target_anguler_pos_deg_);
	rolling_pid.setLimitVal(params_.rolling_vel_auto);
	rolling_pid.update(now_pos_theta_deg);

	if(rolling_pid.getPidEnable())rolling_vel = rolling_pid.getControlVal();

	auto_running_.omni_wheel_.setRunningVector(run_vector.getVector_pol().first, run_vector_pol_local, rolling_vel);

	turn_right_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.turn_right);
	turn_left_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.turn_left);
	turn_pad_was_tilted		= (std::abs(dualshock_.readAnalogPad(params_.buttons_function.turn_control_pad).getVector_rec().first) > 0.1f);
}

void Sequence::mechaControl()
{
	static bool get_bathtowel_was_pushed = false;
	static bool get_t_shirt_was_pushed = false;
	static bool get_sheets_was_pushed = false;
	static bool output_basket_was_pushed = false;
	static bool hang_t_shirt_1_was_pushed = false;
	static bool hang_t_shirt_2_was_pushed = false;
	static bool hang_bathtowel_was_pushed = false;
	static bool increment_mode_was_pushed = false;
	static bool decrement_mode_was_pushed = false;

	static Mode now_mode = Mode::STANDBY;

	uint8_t transmit_data = 0x00;

	static TapeLED tape_led(0x20, can_interface_);
	IO_sigPins<ioName::sig7, ioState::input, pinPullDirection::up> emergency_read;
	if(!emergency_read.readNowState())
	{
		tape_led.setRGB(0, 0, 0);
		return;
	}

	switch(now_mode)
	{
	case Mode::STANDBY:
		tape_led.setRGB(255, 255, 255);
		if(!increment_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.increment_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_T_SHIRT_POINT);
			now_mode = Mode::RECOVERRY;
		}
		break;

	case Mode::RECOVERRY:
		tape_led.setRGB(0, 255, 0);
		if(!increment_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.increment_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_T_SHIRT_POINT);
			now_mode = Mode::HANG_T_SHIRT;
		}
		else if(!decrement_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.decrement_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_ZEROPOINT);
			now_mode = Mode::STANDBY;
		}
		else if(!get_bathtowel_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.get_bathtowel))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GET_BATHTOWEL);
		}
		else if(!get_t_shirt_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.get_T_shirt))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GET_T_SHIRT);
		}
		else if(!get_sheets_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.get_sheets))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GET_SHEETS);
		}
		else if(!output_basket_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.output_basket))
		{
			transmit_data = static_cast<uint8_t>(Telegram::OUTPUT_BASKET);
		}
		break;

	case Mode::HANG_T_SHIRT:
		tape_led.setRGB(0, 0, 255);
		if(!increment_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.increment_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_BATHTOWEL_POINT);
			now_mode = Mode::HANG_BATH_TOWEL;
		}
		else if(!decrement_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.decrement_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_T_SHIRT_POINT);
			now_mode = Mode::RECOVERRY;
		}
		else if(!hang_t_shirt_1_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.hang_T_shirt_1))
		{
			transmit_data = static_cast<uint8_t>(Telegram::HANG_T_SHIRT_1);
		}
		else if(!hang_t_shirt_2_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.hang_T_shirt_2))
		{
			transmit_data = static_cast<uint8_t>(Telegram::HANG_T_SHIRT_2);
		}
		break;

	case Mode::HANG_BATH_TOWEL:
		tape_led.setRGB(255, 0, 0);
		if(!decrement_mode_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.decrement_mode))
		{
			transmit_data = static_cast<uint8_t>(Telegram::GO_T_SHIRT_POINT);
			now_mode = Mode::HANG_T_SHIRT;
		}
		else if(!hang_bathtowel_was_pushed && dualshock_.isButtonPushed(params_.buttons_function.hang_bathtowel))
		{
			transmit_data = static_cast<uint8_t>(Telegram::HANG_BATHTOWEL);
		}
		break;

	default:
		tape_led.setRGB(255, 255, 255);
		break;
	}

	if(transmit_data != 0x00)can_interface_.sendData(&transmit_data, 1, 0x01);

	get_bathtowel_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.get_bathtowel);
	get_t_shirt_was_pushed		= dualshock_.isButtonPushed(params_.buttons_function.get_T_shirt);
	get_sheets_was_pushed		= dualshock_.isButtonPushed(params_.buttons_function.get_sheets);
	output_basket_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.output_basket);
	hang_t_shirt_1_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.hang_T_shirt_1);
	hang_t_shirt_2_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.hang_T_shirt_2);
	hang_bathtowel_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.hang_bathtowel);
	increment_mode_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.increment_mode);
	decrement_mode_was_pushed	= dualshock_.isButtonPushed(params_.buttons_function.decrement_mode);
}
