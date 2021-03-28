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

#include "tosshin_sim/navigation_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <string>
#include <utility>

namespace tosshin_sim
{

const double PI = atan(1) * 4;

NavigationPlugin::NavigationPlugin()
: initial_x_position(0.0),
  initial_y_position(0.0),
  initial_yaw_orientation(0.0),
  forward(0.0),
  left(0.0),
  yaw(0.0)
{
}

void NavigationPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize the node
  {
    node = gazebo_ros::Node::Get(sdf);
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Node initialized with name " << node->get_name() << "!"
    );

    // Initialize the position publisher
    {
      using Position = tosshin_interfaces::msg::Position;
      position_publisher = node->create_publisher<Position>(
        std::string(node->get_name()) + "/position", 10
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Position publisher initialized on " <<
          position_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the orientation publisher
    {
      using Orientation = tosshin_interfaces::msg::Orientation;
      orientation_publisher = node->create_publisher<Orientation>(
        std::string(node->get_name()) + "/orientation", 10
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Orientation publisher initialized on " <<
          orientation_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the maneuver event publisher
    {
      using Maneuver = tosshin_interfaces::msg::Maneuver;
      maneuver_event_publisher = node->create_publisher<Maneuver>(
        std::string(node->get_name()) + "/maneuver_event", 10
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Maneuver event publisher initialized on " <<
          maneuver_event_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the configure maneuver service
    {
      using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;
      configure_maneuver_service = node->create_service<ConfigureManeuver>(
        std::string(node->get_name()) + "/configure_maneuver",
        [this](ConfigureManeuver::Request::SharedPtr request,
        ConfigureManeuver::Response::SharedPtr response) {
          bool configured = false;

          if (request->maneuver.forward.size() > 0) {
            forward = request->maneuver.forward[0];
            response->maneuver.forward.push_back(forward);

            configured = true;
            RCLCPP_DEBUG_STREAM(
              node->get_logger(),
              "Forward maneuver configured into " << forward << "!"
            );
          }

          if (request->maneuver.left.size() > 0) {
            left = request->maneuver.left[0];
            response->maneuver.left.push_back(left);

            configured = true;
            RCLCPP_DEBUG_STREAM(
              node->get_logger(),
              "Left maneuver configured into " << left << "!"
            );
          }

          if (request->maneuver.yaw.size() > 0) {
            yaw = request->maneuver.yaw[0];
            response->maneuver.yaw.push_back(yaw);

            configured = true;
            RCLCPP_DEBUG_STREAM(
              node->get_logger(),
              "Yaw maneuver configured into " << yaw << "!"
            );
          }

          if (configured) {
            maneuver_event_publisher->publish(response->maneuver);
          } else {
            response->maneuver.forward.push_back(forward);
            response->maneuver.left.push_back(left);
            response->maneuver.yaw.push_back(yaw);
          }
        }
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Configure maneuver service initialized on " <<
          configure_maneuver_service->get_service_name() << "!"
      );
    }
  }

  // Initialize the model
  {
    this->model = model;
    world = this->model->GetWorld();

    // Initialize the position and the orientation
    {
      auto pose = model->WorldPose();

      initial_x_position = pose.Pos().X();
      initial_y_position = pose.Pos().Y();
      initial_yaw_orientation = pose.Rot().Yaw();

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Model initialized on position " << initial_x_position <<
          " " << initial_y_position << " and orientation " <<
          initial_yaw_orientation << "!");
    }

    // Initialize the update connection
    {
      last_time = world->SimTime();
      update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&NavigationPlugin::Update, this)
      );

      RCLCPP_INFO(node->get_logger(), "Connected to the world update!");
    }
  }
}

void NavigationPlugin::Update()
{
  auto current_time = world->SimTime();
  auto delta_time = (current_time - last_time).Double();

  // Publish current position
  {
    tosshin_interfaces::msg::Position message;

    auto position = model->WorldPose().Pos();

    message.x = position.X() - initial_x_position;
    message.y = position.Y() - initial_y_position;

    position_publisher->publish(message);
  }

  // Publish current orientation
  {
    tosshin_interfaces::msg::Orientation message;

    message.yaw = model->WorldPose().Rot().Yaw() - initial_yaw_orientation;

    orientation_publisher->publish(message);
  }

  // Set velocities
  {
    auto angle = model->RelativePose().Rot().Yaw();

    auto linear_velocity = ignition::math::Vector3d(
      (forward * cos(angle) + left * sin(angle)) * delta_time * 100.0,
      (forward * sin(angle) + left * cos(angle)) * delta_time * 100.0,
      model->WorldLinearVel().Z()
    );

    model->SetLinearVel(linear_velocity);
    model->SetAngularVel({0.0, 0.0, yaw * delta_time * 100.0});
  }

  // Lock pitch and roll rotations
  {
    auto pose = model->RelativePose();
    auto rot = pose.Rot();

    rot.Euler(0.0, 0.0, rot.Yaw());
    pose.Set(pose.Pos(), rot);

    model->SetRelativePose(pose);
  }

  last_time = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(NavigationPlugin)

}  // namespace tosshin_sim
