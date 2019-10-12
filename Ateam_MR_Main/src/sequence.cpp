/*
 * sequency.cpp
 *
 *  Created on: 2019/10/08
 *      Author: SotaKobayashi
 */

#include "sequence.hpp"

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

	const float run_vector_pol_local = run_vector.getVector_pol().second + (M_PI * 0.5) + auto_running_.dead_reckoning_.pos().posTheta();

	//rolling control logic
	const float rolling_vel = 	dualshock_.isButtonPushed(params_.buttons_function.turn_left) ? -params_.rolling_vel_manual	:
								dualshock_.isButtonPushed(params_.buttons_function.turn_right)? params_.rolling_vel_manual	: 0.0f;
	/*
	 * â…Ç™Ç†Ç¡ÇΩÇÁé¿ëïÇµÇƒÇŸÇµÇ¢ÅAê_Ç…Ç»ÇÍÇÈ
	 *
	static bool turn_right_is_pushed = false;
	static bool turn_left_is_pushed = false;

	constexpr int16_t rigthangle_deg = 90;
	static PID<float, float> rolling_pid(params_.autorunning_params.rolling_gain);
	if(!turn_right_is_pushed && dualshock_.isButtonPushed(params_.buttons_function.turn_right) && dualshock_.isButtonPushed(params_.buttons_function.turn_rightangle_mode))
	{
		target_anguler_pos_deg_ -= rigthangle_deg;
		while(target_anguler_pos_deg_ < -180)target_anguler_pos_deg_ += 180;
		rolling_pid.setPidEnableState(true);
	}
	else if(!turn_left_is_pushed &&  dualshock_.isButtonPushed(params_.buttons_function.turn_left) && dualshock_.isButtonPushed(params_.buttons_function.turn_rightangle_mode))
	{
		target_anguler_pos_deg_ += rigthangle_deg;
		while(target_anguler_pos_deg_ > 180)target_anguler_pos_deg_ -= 180;
		rolling_pid.setPidEnableState(true);
	}
	else if(dualshock_.isButtonPushed(params_.buttons_function.turn_right) && !dualshock_.isButtonPushed(params_.buttons_function.turn_rightangle_mode))
	{

		rolling_pid.setPidEnableState(false);
	}
	else if(dualshock_.isButtonPushed(params_.buttons_function.turn_left) && !dualshock_.isButtonPushed(params_.buttons_function.turn_rightangle_mode))
	{

		rolling_pid.setPidEnableState(false);
	}
	*/

	auto_running_.omni_wheel_.setRunningVector(run_vector.getVector_pol().first, run_vector_pol_local, rolling_vel);
}
