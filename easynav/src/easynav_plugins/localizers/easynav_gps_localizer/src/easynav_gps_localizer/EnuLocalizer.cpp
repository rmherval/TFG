// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Implementation of the GpsLocalizer class.

#include "GpsLocalizerFP.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void GpsLocalizerFP::on_initialize()
{
  auto node = get_node();

  // Subscriber al sensor Fixposition
  fp_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/fixposition/odometry_enu",   // GLOBAL (map frame)
    rclcpp::SensorDataQoS(),
    std::bind(&GpsLocalizerFP::fp_callback, this, std::placeholders::_1));

  // Publisher igual que el plugin original
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    "robot/odom_gps",
    rclcpp::SensorDataQoS());

  // Inicializar odometría
  odom_.header.stamp = node->now();

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  odom_.header.frame_id = tf_info.map_frame;
  odom_.child_frame_id = tf_info.robot_frame;
}

void GpsLocalizerFP::fp_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  fp_odom_ = *msg;
  received_odom_ = true;
}

void GpsLocalizerFP::update(NavState & nav_state)
{
  if (!received_odom_) {
    return;
  }

  // Copiar directamente la odometría del sensor
  odom_ = fp_odom_;

  // Ajustar frames 
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  odom_.header.frame_id = tf_info.map_frame;
  odom_.child_frame_id = tf_info.robot_frame;

  // Guardar en estado global
  nav_state.set("robot_pose", odom_);

  // Publicar
  odom_pub_->publish(odom_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GpsLocalizerFP, easynav::LocalizerMethodBase)