// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TOSSHIN_SIM__MOVEMENT_PLUGIN_HPP_
#define TOSSHIN_SIM__MOVEMENT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/msg/maneuver.hpp>
#include <tosshin_interfaces/srv/configure_maneuver.hpp>

#include <map>
#include <string>

namespace tosshin_sim
{

class MovementPlugin : public gazebo::ModelPlugin
{
public:
  MovementPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void Update();

  double forward;
  double left;
  double yaw;

  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<tosshin_interfaces::msg::Maneuver>::SharedPtr
    maneuver_event_publisher;

  rclcpp::Service<tosshin_interfaces::srv::ConfigureManeuver>::SharedPtr
    configure_maneuver_service;

  gazebo::physics::ModelPtr model;
  gazebo::physics::WorldPtr world;

  gazebo::common::Time last_time;

  gazebo::event::ConnectionPtr update_connection;
};

}  // namespace tosshin_sim

#endif  // TOSSHIN_SIM__MOVEMENT_PLUGIN_HPP_
