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
/// \brief Implementation of the LocalizerNode class.

#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_localizer/LocalizerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

LocalizerNode::LocalizerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("localizer_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  localizer_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::LocalizerMethodBase>>(
    "easynav_core", "easynav::LocalizerMethodBase");
}

LocalizerNode::~LocalizerNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  std::vector<std::string> localizer_types;
  get_parameter("localizer_types", localizer_types);
  for (const auto & localizer_type : localizer_types) {
    localizer_loader_->unloadLibraryForClass(localizer_type);
  }
  localizer_method_ = nullptr;
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
LocalizerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> localizer_types;
  declare_parameter("localizer_types", localizer_types);
  get_parameter("localizer_types", localizer_types);

  if (localizer_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one localizer.  [%lu] found", localizer_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & localizer_type : localizer_types) {
    std::string plugin;
    declare_parameter(localizer_type + std::string(".plugin"), plugin);
    get_parameter(localizer_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading LocalizerMethodBase %s [%s]", localizer_type.c_str(), plugin.c_str());

      localizer_method_ = localizer_loader_->createSharedInstance(plugin);

      try {
        localizer_method_->initialize(shared_from_this(), localizer_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded LocalizerMethodBase %s [%s]", localizer_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin easynav::LocalizerMethodBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
LocalizerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}


bool
LocalizerNode::cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger)
{
  if (localizer_method_ == nullptr) {return false;}

  return localizer_method_->internal_update_rt(*nav_state, trigger);
}

void
LocalizerNode::cycle(std::shared_ptr<NavState> nav_state)
{
  if (localizer_method_ == nullptr) {return;}

  localizer_method_->internal_update(*nav_state);
}

}  // namespace easynav
