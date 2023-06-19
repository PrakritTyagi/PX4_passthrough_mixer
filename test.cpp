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
 * @author Fausto Vega <fausto.vega10@gmail.com>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <chrono>
#include <iostream>
#include <Eigen/Eigen>
#include <fstream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LQR : public rclcpp::Node
{
public:
	LQR() : Node("LQR")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors",10);
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
        vehicle_localposition_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            pos[0] = msg->x;
            pos[1] = msg->y;
            pos[2] = msg->z;

            vel[0] = msg->vx;
            vel[1] = msg->vy;
            vel[2] = msg->vz;
		});

        vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
		[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            quat[0] = msg->q[0];
            quat[1] = msg->q[1];
            quat[2] = msg->q[2];
            quat[3] = msg->q[3];
		});

        vehicle_angularvelocity_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
		[this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
            ang_vel[0] = msg->xyz[0];
            ang_vel[1] = msg->xyz[1];
            ang_vel[2] = msg->xyz[2];
            
		});


		data_store.open ("/home/prakrit/ws_px4/src/px4_ros_com/src/examples/data/deltax.txt");
		RCLCPP_INFO(this->get_logger(), "File Opened");


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
 
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				std::cout<<"Changed to Offboard mode after 10 setpoints"<<std::endl;

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with publish_actuator_motors
			publish_offboard_control_mode();
			Eigen::Matrix<double,12,1> state_error_ = get_state_error();
			publish_actuator_motors(state_error_);

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				std::cout<<offboard_setpoint_counter_<<std::endl;
				offboard_setpoint_counter_++;
			}
		};
		//Initializing hover point

		this->pos0<<0,0,-1.0;
		this->quat0<<0.6922445297241211, 0.001254965434782207, -0.01065171230584383, 0.721583366394043;
		this->vel0<<0,0,0;
		this->ang_vel0<<0,0,0;
		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;

// Subscriptions
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_localposition_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr vehicle_angularvelocity_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_actuator_motors(Eigen::Matrix<double,12,1>& state_error);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	Eigen::Matrix<double,12,1> get_state_error();
	Eigen::Vector3d get_qtorp(const Eigen::Vector4d& q);
	Eigen::Matrix4d get_L(const Eigen::Vector4d& q);
	Eigen::Matrix3d get_hat(const Eigen::Vector3d& vec);
	Eigen::Matrix3d qtoQ(const Eigen::Vector4d& q);

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
	
	std::ofstream data_store;
};
// -----------------------------------------------------------------------------------//
// Function declerations //

Eigen::Matrix3d LQR::get_hat(const Eigen::Vector3d& vec)
{
	Eigen::Matrix3d hat = Eigen::Matrix3d::Zero();
	hat(0,1) = -vec(2);
	hat(0,2) = vec(1);
	hat(1,0) = vec(2);
	hat(1,2) = -vec(0);
	hat(2,0) = -vec(1);
	hat(2,1) = vec(0);
	return hat;
}

Eigen::Matrix4d LQR::get_L(const Eigen::Vector4d& q)
{
	double s = q(0);
	Eigen::Vector3d v = q.tail<3>();
	Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();

	Eigen::Matrix4d L;
	L.block<3,3>(1,1) = s*eye + this->get_hat(v);
	L.block<4,1>(0,0) = q;
	L.block<1,3>(0,1) = -v;
	return L;
}

Eigen::Vector3d LQR::get_qtorp(const Eigen::Vector4d& q)
{	
	Eigen::Vector3d v = q.tail<3>()/q(0);
	return v;
}

Eigen::Matrix3d LQR::qtoQ(const Eigen::Vector4d& q)
{
	Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
	Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
	T(0,0) = 1.0;
	T.block<3,3>(1,1) = -1*eye;

	Eigen::Matrix<double,4,3> H = Eigen::Matrix<double,4,3>::Zero();
	H.block<3,3>(1,0) = eye;

	Eigen::Matrix4d L = this->get_L(q);
	Eigen::Matrix3d Q = H.transpose()*T*L*T*L*H;

	return Q;

}

Eigen::Matrix<double,12,1> LQR::get_state_error()
{	
	Eigen::Vector3d del_pos = this->pos - this->pos0;

	Eigen::Matrix3d Q = this->qtoQ(quat);
	Eigen::Vector3d body_vel = Q.transpose()*this->vel;
	Eigen::Vector3d del_vel = body_vel - this->vel0;

	Eigen::Vector3d del_ang_vel = this->ang_vel - this->ang_vel0;

	// Calculating del_quat
	Eigen::Vector3d rp = get_qtorp(get_L(quat0).transpose()*quat); 

	Eigen::Matrix<double,12,1> state_error;
	state_error<<del_pos,rp,del_vel,del_ang_vel;
	// std::cout<<"state error: "<<state_error<<std::endl;

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
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void LQR::publish_actuator_motors(Eigen::Matrix<double,12,1>& state_error)
{
	// double value= 0.71;
	// Eigen::Vector4d uhover = Eigen::Vector4d::Constant(value); 
	double mass = 1.535;
	double gravity = 9.81;
	double hover_value = (mass*gravity/4);
	Eigen::Vector4d uhover = Eigen::Vector4d::Constant(hover_value); 

	Eigen::Matrix<double,4,12> K;
	K<<1.142603478590523, -1.234103898553352, -1.5192644205648596, -13.267987477125917, 13.486260589016307, 1.3734159520849187, -1.7241317277225028, -1.7144536680111246, -1.8927967952925069, -1.3298270062905109, 1.331809603704471, 1.8016576467424739,
	 -1.1462030611948444, 1.1840317929456206, -1.5642316694072365, 13.519075115496365,-13.302470475801922, 1.7437733662455919, 1.736350734303218, 1.7459520075277462, -1.8813004359879308, 1.332157782078033, -1.3301904164558271, 1.830038251717522,
	 -1.1972811768805773, -1.1664057198313758, -1.5281085350470647, 13.239299662901544, 13.417434333898218, -1.2533425994800196, -1.7175270614311842, 1.7190374185545918, -1.890535702595963, 1.329580841270191, 1.3311756892715751, -1.792456388033368,
	  1.2212141746231535, 1.1222765415711986 ,-1.5553875549185696, -13.546893841758807, -13.370427643280816, -1.8638467188659873, 1.7429154034323175, -1.7413282598102071, -1.88356152868304, -1.3323960130330834, -1.3308163968266502, -1.839239510428769;

	Eigen::Vector4d output = uhover -K*state_error;
	std::cout<<"uhover: "<<std::endl<<uhover<<std::endl;
	std::cout<<"output: "<<std::endl<<output<<std::endl;

	// for (int i = 0; i <= 3; i++){
	// 	if (output(i) >= 2.0 || output(i) <= 0){
	// 		if (output(i) >= 2.0){
	// 			output(i) = 2.0;
	// 		}else{
	// 			output(i) = 0.0;
	// 		}
	// 	}
	// }

	double max_thrust = 5.5;
	Eigen::Vector4d norm_output = output/max_thrust;
	for (int i = 0; i <= 3; i++){
		if (norm_output(i) >= 1.0 || norm_output(i) <= 0){
			if (norm_output(i) >= 1.0){
				norm_output(i) = 1.0;
			}else{
				norm_output(i) = 0.0;
			}
		}
	}

	if (data_store.is_open())
		{
			data_store << state_error<<"\n";
			// myfile.close();
		}

	// Eigen::Vector4d u = uhover + norm_output;
	ActuatorMotors msg{};
	msg.control[0] = norm_output(0);
	msg.control[1] = norm_output(1);
	msg.control[2] = norm_output(2);
	msg.control[3] = norm_output(3);
	msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_motors_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void LQR::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LQR>());

	rclcpp::shutdown();
	return 0;
}