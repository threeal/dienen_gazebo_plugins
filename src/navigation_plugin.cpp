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
#include <keisan/keisan.hpp>

#include <algorithm>

namespace tosshin_gazebo_plugins
{

NavigationPlugin::NavigationPlugin()
{
}

void NavigationPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize the node
  set_node(gazebo_ros::Node::Get(sdf));

  // Initialize the model
  this->model = model;

  // Initialize the position and the orientation
  {
    auto pose = model->WorldPose();

    initial_odometry.position.x = pose.Pos().X();
    initial_odometry.position.y = pose.Pos().Y();
    initial_odometry.orientation.yaw = pose.Rot().Yaw();

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Model initialized on position " << initial_odometry.position.x <<
        " " << initial_odometry.position.y << " and orientation " <<
        initial_odometry.orientation.yaw << "!");
  }

  // Initialize the update connection
  {
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&NavigationPlugin::Update, this)
    );

    RCLCPP_INFO(get_node()->get_logger(), "Connected to the world update!");
  }
}

void NavigationPlugin::Update()
{
  // Update current odometry
  {
    tosshin_cpp::Odometry odometry;

    auto pos = model->WorldPose().Pos();

    odometry.position.x = pos.X() - initial_odometry.position.x;
    odometry.position.y = pos.Y() - initial_odometry.position.y;

    double yaw = model->WorldPose().Rot().Yaw();

    odometry.orientation.yaw = keisan::rad_to_deg(yaw - initial_odometry.orientation.yaw);

    set_odometry(odometry);
  }

  // Update velocities
  {
    auto maneuver = get_maneuver();

    auto angle = model->RelativePose().Rot().Yaw();
    auto gravity = model->WorldLinearVel().Z();

    auto linear_velocity = ignition::math::Vector3d(
      (maneuver.forward * cos(angle) + maneuver.left * sin(angle)) / 60.0,
      (maneuver.forward * sin(angle) + maneuver.left * cos(angle)) / 60.0,
      std::min(gravity, 0.0)
    );

    model->SetLinearVel(linear_velocity);
    model->SetAngularVel({0.0, 0.0, maneuver.yaw / 60.0});
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

GZ_REGISTER_MODEL_PLUGIN(NavigationPlugin)

}  // namespace tosshin_gazebo_plugins
