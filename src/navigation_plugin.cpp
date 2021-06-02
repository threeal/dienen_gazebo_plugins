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

#include "tosshin_gazebo_plugins/navigation_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <algorithm>

namespace tosshin_gazebo_plugins
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

  // Initialize the initial pose
  initial_pose = get_pose();

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
      (linear.x * cos(angle) + linear.y * sin(angle)) / 60.0,
      (linear.x * sin(angle) + linear.y * cos(angle)) / 60.0,
      std::min(gravity, 0.0)
    );

    model->SetLinearVel(linear_velocity);
    model->SetAngularVel({0.0, 0.0, current_twist.angular.z / 60.0});
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

Pose NavigationPlugin::get_pose() const
{
  Pose pose;

  auto pos = model->WorldPose().Pos();
  pose.position.x = pos.X();
  pose.position.y = pos.Y();
  pose.position.z = pos.Z();

  auto rot = model->WorldPose().Rot();
  pose.orientation.x = rot.X();
  pose.orientation.y = rot.Y();
  pose.orientation.z = rot.Z();
  pose.orientation.w = rot.W();

  return pose;
}

Odometry NavigationPlugin::get_odometry() const
{
  Odometry odometry;

  auto sim_time = model->GetWorld()->SimTime();
  odometry.header.stamp.sec = sim_time.sec;
  odometry.header.stamp.nanosec = sim_time.nsec;

  odometry.header.frame_id = "odom";

  auto pose = get_pose();

  pose.position.x -= initial_pose.position.x;
  pose.position.y -= initial_pose.position.y;
  pose.position.z -= initial_pose.position.z;

  pose.orientation.x -= initial_pose.orientation.x;
  pose.orientation.y -= initial_pose.orientation.y;
  pose.orientation.z -= initial_pose.orientation.z;
  pose.orientation.w -= initial_pose.orientation.w;

  odometry.pose.pose = pose;
  odometry.twist.twist = current_twist;

  return odometry;
}

GZ_REGISTER_MODEL_PLUGIN(NavigationPlugin)

}  // namespace tosshin_gazebo_plugins
