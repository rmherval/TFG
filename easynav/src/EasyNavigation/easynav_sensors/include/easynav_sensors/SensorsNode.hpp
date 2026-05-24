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
/// \brief Declaration of the SensorsNode class, a ROS 2 lifecycle node for sensor fusion tasks in Easy Navigation.

#ifndef EASYNAV_SENSORS__SENSORNODE_HPP_
#define EASYNAV_SENSORS__SENSORNODE_HPP_

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class SensorsNode
 * @brief ROS 2 lifecycle node that manages sensor fusion in Easy Navigation.
 *
 * Collects, transforms, and publishes fused perception data from multiple sources.
 */
class SensorsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SensorsNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Function pointer type used to handle sensor groups
   * @param group Sensor group name.
   * @param perceptions List of perceptions (one element from each sensor in the group).
   * @param ns Navigation state to populate.
   */
  using SensorsHandlerFn = void(*)(
    const std::string & group,
    const std::vector<easynav::PerceptionPtr> & perceptions,
    ::easynav::NavState & ns
  );

  /**
   * @brief Constructor.
   * @param options Node configuration options.
   */
  explicit SensorsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~SensorsNode();

  /**
   * @brief Configure the node.
   * @param state Lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup the node.
   * @param state Lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition errors.
   * @param state Lifecycle state.
   * @return SUCCESS if error was handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the callback group for real-time tasks.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Run one real-time sensor processing cycle.
   * @param trigger Force execution regardless of frequency.
   * @return True if cycle executed.
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger = false);

  /**
   * @brief Run one non-real-time processing cycle.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

  void register_handler(std::shared_ptr<PerceptionHandler> handler);

private:
  /// @brief Callback group for real-time operations.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Publisher for the fused perception point cloud.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr percept_pub_;

  /// @brief Last fused perception message.
  sensor_msgs::msg::PointCloud2 perecption_msg_;

  /// @brief Maximum time (seconds) a perception remains valid.
  double forget_time_;

  /// @brief Target frame for perception fusion.
  std::string robot_frame_ {"base_link"};

  /// @brief Target frame for perception fusion.
  std::string tf_prefix_;

  /// @brief Shared pointer to the navigation state structure.
  std::shared_ptr<NavState> nav_state_;

  std::map<std::string, std::vector<PerceptionPtr>> perceptions_;
  std::map<std::string, std::shared_ptr<PerceptionHandler>> handlers_;

  /// @brief Map from group name to handler function pointer (for fast dispatch in set_by_group)
  std::unordered_map<std::string, SensorsHandlerFn> group_to_handler_;

  /**
   * @brief Populate NavState for a given group from perceptions; returns true if handled
   * @param group Sensor group name.
   * @param perceptions Vector of perceptions (one element from each sensor in the group).
   * @param ns Navigation state to populate.
   */
  bool set_by_group(
    const std::string & group,
    const std::vector<PerceptionPtr> & perceptions,
    ::easynav::NavState & ns
  );

  /**
   * @brief Populate the runtime map group_to_handler_.
   * This method uses compile-time template recursion to fill the map
   * with the sensor handler functions for the default groups.
   */
  template<std::size_t I = 0>
  void populate_group_to_handler_map();
};

}  // namespace easynav

#endif  // EASYNAV_SENSORS__SENSORNODE_HPP_
