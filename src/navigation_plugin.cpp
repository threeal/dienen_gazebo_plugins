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

#include "dienen_gazebo_plugins/navigation_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <algorithm>

namespace dienen_gazebo_plugins
{

NavigationPlugin::NavigationPlugin()
{
}

void NavigationPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize the node
  {
    node = gazebo_ros::Node::Get(sdf);

    // Initialize the velocity subscription
    twist_subscription = node->create_subscription<Twist>(
      "/cmd_vel", 10,
      [&](const Twist::SharedPtr msg) {
        current_twist = *msg;
      });

    // Initialize the odometry publisher
    odometry_publisher = node->create_publisher<Odometry>("/odom", 10);
  }

  this->model = model;

  // Initialize the initial position
  initial_position = get_position();
  initial_orientation = get_orientation();

  // Initialize the update connection
  update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&NavigationPlugin::Update, this)
  );

  RCLCPP_INFO(node->get_logger(), "Connected to the world update!");
}

void NavigationPlugin::Update()
{
  // Publish current odometry
  auto odometry = get_odometry();
  odometry_publisher->publish(odometry);

  // Update velocities
  {
    auto angle = model->RelativePose().Rot().Yaw();
    auto gravity = model->WorldLinearVel().Z();

    auto linear = current_twist.linear;
    auto linear_velocity = ignition::math::Vector3d(
      linear.x * cos(angle) - linear.y * sin(angle),
      linear.x * sin(angle) + linear.y * cos(angle),
      std::min(gravity, 0.0)
    );

    model->SetLinearVel(linear_velocity);
    model->SetAngularVel({0.0, 0.0, current_twist.angular.z});
  }

  // Lock pitch and roll rotations
  {
    auto pose = model->RelativePose();
    auto rot = pose.Rot();

    rot.Euler(0.0, 0.0, rot.Yaw());
    pose.Set(pose.Pos(), rot);

    model->SetRelativePose(pose);
  }
}

keisan::Point2 NavigationPlugin::get_position() const
{
  auto pos = model->WorldPose().Pos();
  return keisan::Point2(pos.X(), pos.Y());
}

keisan::Angle NavigationPlugin::get_orientation() const
{
  return keisan::make_radian(model->WorldPose().Rot().Yaw());
}

Odometry NavigationPlugin::get_odometry() const
{
  Odometry odometry;

  auto sim_time = model->GetWorld()->SimTime();
  odometry.header.stamp.sec = sim_time.sec;
  odometry.header.stamp.nanosec = sim_time.nsec;

  odometry.header.frame_id = "odom";

  auto position = get_position();
  auto orientation = get_orientation();

  // transform current position and orientation according to the initial position and orientation
  position = position.translate(-initial_position).rotate(-initial_orientation);
  orientation = initial_orientation.difference_to(orientation);

  odometry.pose.pose.position.x = position.x;
  odometry.pose.pose.position.y = position.y;
  odometry.pose.pose.position.z = 0.0;

  auto quaternion = keisan::EulerAngles(
    keisan::make_degree(0.0), keisan::make_degree(0.0), orientation).quaternion();

  odometry.pose.pose.orientation.x = quaternion.x;
  odometry.pose.pose.orientation.y = quaternion.y;
  odometry.pose.pose.orientation.z = quaternion.z;
  odometry.pose.pose.orientation.w = quaternion.w;

  odometry.twist.twist = current_twist;

  return odometry;
}

GZ_REGISTER_MODEL_PLUGIN(NavigationPlugin)

}  // namespace dienen_gazebo_plugins
