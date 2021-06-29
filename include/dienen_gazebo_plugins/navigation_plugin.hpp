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

#ifndef DIENEN_GAZEBO_PLUGINS__NAVIGATION_PLUGIN_HPP_
#define DIENEN_GAZEBO_PLUGINS__NAVIGATION_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin/tosshin.hpp>

namespace dienen_gazebo_plugins
{

class NavigationPlugin : public gazebo::ModelPlugin
{
public:
  NavigationPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void Update();

  keisan::Point2 get_position() const;
  keisan::Angle get_orientation() const;

  tosshin::msg::Odometry get_odometry() const;

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<tosshin::msg::Twist>::SharedPtr twist_subscription;
  rclcpp::Publisher<tosshin::msg::Odometry>::SharedPtr odometry_publisher;
  rclcpp::Publisher<tosshin::msg::TFMessage>::SharedPtr tf_publisher;

  keisan::Point2 initial_position;
  keisan::Angle initial_orientation;

  tosshin::msg::Twist current_twist;

  double linear_speed_scale;
  double angular_speed_scale;

  gazebo::physics::ModelPtr model;

  gazebo::event::ConnectionPtr update_connection;
};

}  // namespace dienen_gazebo_plugins

#endif  // DIENEN_GAZEBO_PLUGINS__NAVIGATION_PLUGIN_HPP_
