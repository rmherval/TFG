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
/// \brief Declaration of the GpsLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__GPSLOCALIZER_HPP_

#pragma once

#include "easynav_core/LocalizerMethodBase.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace easynav
{

class GpsLocalizerFP : public LocalizerMethodBase
{
public:
  void on_initialize() override;
  void update(NavState & nav_state) override;
  void update_rt(NavState & nav_state) override;

private:
  void fp_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fp_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  nav_msgs::msg::Odometry fp_odom_;
  nav_msgs::msg::Odometry odom_;

  bool received_odom_{false};
};

}  // namespace easynav