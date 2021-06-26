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

#include <dienen_gazebo_plugins/navigation_plugin.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <algorithm>

namespace ksn = keisan;
namespace tsn = tosshin;

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
    twist_subscription = node->create_subscription<tsn::msg::Twist>(
      "/cmd_vel", 10,
      [&](const tsn::msg::Twist::SharedPtr msg) {
        current_twist = *msg;
      });

    // Initialize the odometry publisher
    odometry_publisher = node->create_publisher<tsn::msg::Odometry>("/odom", 10);
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
    auto angle = ksn::make_radian(model->RelativePose().Rot().Yaw());
    auto gravity = model->WorldLinearVel().Z();

    auto linear_vel = tsn::extract_vector3_xy(current_twist.linear).rotate(angle);

    model->SetLinearVel({linear_vel.x, linear_vel.y, std::min(gravity, 0.0)});
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

ksn::Point2 NavigationPlugin::get_position() const
{
  auto pos = model->WorldPose().Pos();
  return ksn::Point2(pos.X(), pos.Y());
}

ksn::Angle NavigationPlugin::get_orientation() const
{
  return ksn::make_radian(model->WorldPose().Rot().Yaw());
}

tsn::msg::Odometry NavigationPlugin::get_odometry() const
{
  tsn::msg::Odometry odometry;

  auto sim_time = model->GetWorld()->SimTime();
  odometry.header.stamp.sec = sim_time.sec;
  odometry.header.stamp.nanosec = sim_time.nsec;

  odometry.header.frame_id = "odom";

  auto position = get_position();
  auto orientation = get_orientation();

  // transform current position and orientation according to the initial position and orientation
  position = position.translate(-initial_position).rotate(-initial_orientation);
  orientation = initial_orientation.difference_to(orientation);

  auto euler = ksn::EulerAngles(ksn::make_degree(0.0), ksn::make_degree(0.0), orientation);

  odometry.pose.pose.position = tsn::make_point_xy(position);
  odometry.pose.pose.orientation = tsn::make_quaternion(euler.quaternion());

  odometry.twist.twist = current_twist;

  return odometry;
}

GZ_REGISTER_MODEL_PLUGIN(NavigationPlugin)

}  // namespace dienen_gazebo_plugins
