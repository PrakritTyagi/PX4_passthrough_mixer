/****************************************************************************
 *
 * Copyright 2018 PX4 Development Team. All rights reserved.
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
 * @brief Actuator Motors uORB topic adverstiser 
 * @file actuator_publisher.cpp
 * @addtogroup examples
 * @author Fausto Vega <fvega@andrew.cmu.edu>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <px4_msgs/msg/actuator_motors.hpp> //Add the actuator motors message
#include <px4_msgs/msg/timesync.hpp> //add time sync message


using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class ActuatorAdvertiser : public rclcpp::Node
{
public:
	ActuatorAdvertiser() : Node("actuator_advertiser") {
#ifdef ROS_DEFAULT_API
        actuator_pub_= this->create_publisher<px4_msgs::msg::ActuatorMotors>("fmu/actuator_motors/in", 10);

#else
        actuator_pub_= this->create_publisher<px4_msgs::msg::ActuatorMotors>("fmu/actuator_motors/in");

#endif

		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});


		auto timer_callback =
		[this]()->void {
            auto actuator_motors = px4_msgs::msg::ActuatorMotors();
			actuator_motors.timestamp =  timestamp_.load();
            //actuator_motors.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            //actuator_motors.timestamp_sample = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			actuator_motors.control[0] = 0.15;
            actuator_motors.control[1] = 0.15;
            actuator_motors.control[2] = 0.15;
            actuator_motors.control[3] = 0.15;

            this->actuator_pub_->publish(actuator_motors);
		};

        timer_ = this->create_wall_timer(10ms, timer_callback); //running at 100 Hz
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_pub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_motors advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
