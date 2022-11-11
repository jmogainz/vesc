// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/twist_to_ackermann.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using ackermann_msgs::msg::AckermannDrive;
using geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std_msgs::msg::Float64;

TwistToAckermann::TwistToAckermann(const rclcpp::NodeOptions & options)
: Node("twist_to_ackermann_node", options)
{
  // get conversion parameters
  declare_parameter("twist_cmd_topic", twist_cmd_topic_);
  declare_parameter("ackermann_cmd_topic", ackermann_cmd_topic_);
  declare_parameter("wheelbase", wheelbase_);
  declare_parameter("frame_id", frame_id_);

  // create publishers to ackermann_cmd topic
  ackermann_cmd_pub_ = create_publisher<AckermannDriveStamped>(ackermann_cmd_topic_, 10);

  // subscribe to ackermann topic
  twist_cmd_sub_ = create_subscription<Twist>(
    twist_cmd_topic_, 10, std::bind(&TwistToAckermann::twistCmdCallback, this, _1));
}

double convert_trans_rot_vel_to_steering_angle(
  double v, double omega, double wheelbase)
{
  if (omega == 0 || v == 0) {
    return 0;
  }

  double radius = v / omega;
  return std::atan(wheelbase / radius);
}

void TwistToAckermann::twistCmdCallback(const Twist::SharedPtr cmd)
{
  double v = cmd->linear.x;
  double steering = convert_trans_rot_vel_to_steering_angle(v, cmd->angular.z, wheelbase_);

  AckermannDriveStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = now();
  msg.drive.speed = v;
  msg.drive.steering_angle = steering;

  if (rclcpp::ok()) {
    ackermann_cmd_pub_->publish(msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::TwistToAckermann)
