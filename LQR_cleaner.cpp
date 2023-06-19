/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief LQR example
 * @file LQR.cpp
 * @addtogroup examples
 * @author Prakrit Tyagi <prakritt@andrew.cmu.edu> <tyagiprakrit@gmail.com>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <chrono>
#include <iostream>
#include <fstream>
#include "fmt/core.h"
#include "Eigen/Dense"
#include "altro/altro.hpp"

#include <px4_ros_com/quaternion_math.h>
#include <px4_ros_com/dynamics_mc.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LQR : public rclcpp::Node
{
public:
	LQR() : Node("LQR")
	{
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		actuator_motors_publisher_ =
			this->create_publisher<ActuatorMotors>("fmu/actuator_motors/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
		actuator_motors_publisher_ =
			this->create_publisher<ActuatorMotors>("fmu/actuator_motors/in");
#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
															   [this](const px4_msgs::msg::Timesync::UniquePtr msg)
															   {
																   timestamp_.store(msg->timestamp);
															   });

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Add subscribers here for the data you want to subscribe to
		vehicle_localposition_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/vehicle_local_position/out", qos,
																										   [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
																										   {
																											   pos[0] = msg->x;
																											   pos[1] = msg->y;
																											   pos[2] = msg->z;

																											   vel[0] = msg->vx;
																											   vel[1] = msg->vy;
																											   vel[2] = msg->vz;
																										   });

		vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/vehicle_attitude/out", qos,
																								 [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
																								 {
																									 quat[0] = msg->q[0];
																									 quat[1] = msg->q[1];
																									 quat[2] = msg->q[2];
																									 quat[3] = msg->q[3];
																								 });

		vehicle_angularvelocity_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/vehicle_angular_velocity/out", qos,
																											   [this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg)
																											   {
																												   ang_vel[0] = msg->xyz[0];
																												   ang_vel[1] = msg->xyz[1];
																												   ang_vel[2] = msg->xyz[2];
																											   });

		data_store.open("/home/prakrit/px4_ros_com_ros2/src/px4_ros_com/src/examples/data/deltax.txt");
		std::cout << "File Opened" << std::endl;

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void
		{
			if (offboard_setpoint_counter_ == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				std::cout << "Changed to Offboard mode after 10 setpoints" << std::endl;

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with publish_actuator_motors
			publish_offboard_control_mode();
			Eigen::Matrix<double, 12, 1> state_error_ = get_state_error();
			publish_actuator_motors(state_error_);

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11)
			{
				std::cout << "Starting in " << offboard_setpoint_counter_ << std::endl;
				offboard_setpoint_counter_++;
			}
		};

		// Initializing hover point

		this->pos0 << 0.0, 0.0, -1.0;
		this->quat0 << 0.7167351841926575, -0.008279332891106606, 0.0051797619089484215, 0.6972771286964417;
		this->vel0 << 0.0, 0.0, 0.0;
		this->ang_vel0 << 0.0, 0.0, 0.0;

		// Calculating A and B matrices

		this->Ak = dynamics_mc::get_A(pos0, quat0, vel0, ang_vel0);
		this->Bk = dynamics_mc::get_B();

		this->Ak_ = quaternion_math::get_E(this->quat0).transpose() * this->Ak * quaternion_math::get_E(this->quat0);
		this->Bk_ = quaternion_math::get_E(this->quat0).transpose() * this->Bk;

		// Setting weights for LQR
		this->Qvec << 1000000, 1000000, 1000000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000;
		this->Svec << 1000000, 1000000, 1000000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000;
		this->Rvec << 10000, 10000, 10000, 10000;

		this->set_K_ricatti(this->Ak_, this->Bk_, this->Qvec, this->Svec, this->Rvec);

		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_localposition_subscriber_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angularvelocity_subscriber_;

	std::atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_actuator_motors(Eigen::Matrix<double, 12, 1> &state_error);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
								 float param2 = 0.0);

	Eigen::Matrix<double, 12, 1> get_state_error();
	void set_K_ricatti(const Eigen::Matrix<double, 12, 12> &A,
					   const Eigen::Matrix<double, 12, 4> &B,
					   const Eigen::Matrix<double, 12, 1> &Qvec,
					   const Eigen::Matrix<double, 12, 1> &Svec,
					   const Eigen::Vector4d &Rvec);

	//  state variables
	Eigen::Vector3d pos;
	Eigen::Vector4d quat;
	Eigen::Vector3d vel;
	Eigen::Vector3d ang_vel;
	//  linearized point
	Eigen::Vector3d pos0;
	Eigen::Vector4d quat0;
	Eigen::Vector3d vel0;
	Eigen::Vector3d ang_vel0;
	//  linearized A and B matrices
	Eigen::Matrix<double, 13, 13> Ak;
	Eigen::Matrix<double, 13, 4> Bk;
	Eigen::Matrix<double, 12, 12> Ak_;
	Eigen::Matrix<double, 12, 4> Bk_;
	//  LQR gain
	Eigen::Matrix<double, 4, 12> K;
	//  LQR weights
	Eigen::Matrix<double, 12, 1> Qvec;
	// Eigen::Vector<double, 12> Qvec;
	Eigen::Matrix<double, 12, 1> Svec;
	// Eigen::Vector<double, 12> Svec;
	Eigen::Vector4d Rvec;

	std::ofstream data_store;
};
// -----------------------------------------------------------------------------------//

void LQR::set_K_ricatti(const Eigen::Matrix<double, 12, 12> &A,
						const Eigen::Matrix<double, 12, 4> &B,
						const Eigen::Matrix<double, 12, 1> &Qvec,
						const Eigen::Matrix<double, 12, 1> &Svec,
						const Eigen::Vector4d &Rvec)
{
	Eigen::Matrix<double, 12, 12> Q = Qvec.asDiagonal();
	Eigen::Matrix<double, 12, 12> S = Svec.asDiagonal();
	Eigen::Matrix<double, 4, 4> R = Rvec.asDiagonal();
	Eigen::Matrix<double, 4, 12> Kold = Eigen::Matrix<double, 4, 12>::Zero();
	for (int i = 0; i < 2000; i++)
	{
		this->K = (R + B.transpose() * S * B).colPivHouseholderQr().solve(B.transpose() * S * A);
		S = Q + this->K.transpose() * R * this->K + (A - B * this->K).transpose() * S * (A - B * this->K);
		if ((K - Kold).cwiseAbs().maxCoeff() < 1e-7)
		{
			// cwiseAbs() returns an expression of the coefficient-wise absolute value
			std::cout << "Converged in " << i << " iterations" << std::endl;
			break;
		}
		Kold = this->K;
	}
	std::cout << "K: " << std::endl
			  << this->K << std::endl;
}

Eigen::Matrix<double, 12, 1> LQR::get_state_error()
{
	Eigen::Vector3d del_pos = this->pos - this->pos0;

	Eigen::Matrix3d Q = quaternion_math::qtoQ(quat);
	Eigen::Vector3d body_vel = Q.transpose() * this->vel;
	Eigen::Vector3d del_vel = body_vel - this->vel0;

	Eigen::Vector3d del_ang_vel = this->ang_vel - this->ang_vel0;

	Eigen::Vector3d rp = quaternion_math::get_qtorp(quaternion_math::get_L(quat0).transpose() * quat);

	Eigen::Matrix<double, 12, 1> state_error;
	state_error << del_pos, rp, del_vel, del_ang_vel;

	return state_error;
}
/**
 * @brief Send a command to Arm the vehicle
 */
void LQR::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void LQR::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void LQR::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = true;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void LQR::publish_actuator_motors(Eigen::Matrix<double, 12, 1> &state_error)
{

	double mass = 1.535;
	double gravity = 9.81;
	double hover_value = (mass * gravity / 4);
	Eigen::Vector4d uhover = Eigen::Vector4d::Constant(hover_value);

	// Eigen::Matrix<double, 4, 12> Kjulia;
	// Kjulia << 4.265895064228287, -4.559273004970048, -4.8017992850569176, -10.604971718143497, 10.665611730109505, 0.34871330395097694, -2.2016703126510837, -2.2133882242511635, -2.0037069239655443, -0.7031671984208737, 0.7045584274273765, 0.716384234727641,
	// 	-4.249603688921086, 4.437644575798168, -4.937463991307867, 10.641626949527582, -10.581762865048395, 0.6461134568524391, 2.241683447573452, 2.2299583752968353, -1.983637384357718, 0.7042106074004072, -0.7028346669202661, 0.7353727272198742,
	// 	-4.481280268966817, -4.3163410407369085, -4.828481699588452, 10.587472225769117, 10.652200116973358, -0.2522929051831694, -2.200103726443246, 2.223565585203066, -1.9997596900224348, 0.7028240037333324, 0.7042159298132408, -0.7102274779060819,
	// 	4.516608094620598, 4.198821782165929, -4.910781667650304, -10.658716615974244, -10.59476880892584, -0.742534083075435, 2.2432460812799087, -2.219776272932335, -1.987584644387287, -0.7045457151593977, -0.7031691667386308, -0.7415298509709453;
	Eigen::Vector4d output = uhover - this->K * state_error;

	// Normalize output
	double max_thrust = 7.0; // 5.5
	Eigen::Vector4d norm_output = output / max_thrust;
	for (int i = 0; i <= 3; i++)
	{
		if (norm_output(i) >= 1.0 || norm_output(i) <= 0)
		{
			if (norm_output(i) >= 1.0)
			{
				norm_output(i) = 1.0;
			}
			else
			{
				norm_output(i) = 0.0;
			}
		}
	}

	// std::cout << "norm_output: " << std::endl
	// 		  << norm_output << std::endl;
	// std::cout << "output: " << std::endl
	// 		  << output << std::endl;

	if (data_store.is_open())
	{
		data_store << state_error << "\n";
		// myfile.close();
	}

	ActuatorMotors msg{};
	msg.timestamp = timestamp_.load();
	msg.control[0] = norm_output(0);
	msg.control[1] = norm_output(1);
	msg.control[2] = norm_output(2);
	msg.control[3] = norm_output(3);

	actuator_motors_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void LQR::publish_vehicle_command(uint16_t command, float param1,
								  float param2)
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << R"(
	__/\\\\\\\\\____________/\\\___________/\\\\\\\\\_____ 
	 _\///\\\\\/__________/\\\\/\\\\______/\\\///////\\\___ 
	  ___\/\\\___________/\\\//\////\\\___\/\\\_____\/\\\___  
	   ___\/\\\__________/\\\______\//\\\__\/\\\\\\\\\\\/____ 
	    ___\/\\\_________\//\\\______/\\\___\/\\\//////\\\____ 
	     ___\/\\\__________\///\\\\/\\\\/____\/\\\____\//\\\___ 
	      ___\/\\\____________\////\\\//______\/\\\_____\//\\\__  
	       __/\\\\\\\\\\\\________\///\\\\\\___\/\\\______\//\\\_
	        _\////////////___________\//////____\///________\///__ 

	)" << '\n';

	std::cout << "Starting LQR control node for ......" << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LQR>());

	rclcpp::shutdown();
	return 0;
}
