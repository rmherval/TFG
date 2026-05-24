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
/// \brief Implementation of the SensorsNode class.

#include <tuple>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <array>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/SensorsNode.hpp"

#include "easynav_common/types/ImagePerception.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/types/IMUPerception.hpp"
#include "easynav_common/types/GNSSPerception.hpp"
#include "easynav_common/types/DetectionsPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

/// Anonymous namespace for helper functions
namespace
{

// Compile-time Perception type registry
using Registry = std::tuple<
  easynav::ImagePerception,
  easynav::IMUPerception,
  easynav::GNSSPerception,
  easynav::PointPerception,
  easynav::DetectionsPerception
>;

template<std::size_t I = 0>
std::string resolve_group_from_msg(std::string_view msg_type)
{
  if constexpr (I == std::tuple_size_v<Registry>) {
    return {};
  } else {
    using P = std::tuple_element_t<I, Registry>;
    if (P::supports_msg_type(msg_type)) {
      return std::string(P::default_group_);
    }
    return resolve_group_from_msg<I + 1>(msg_type);
  }
}

/**
 * Specialized handler function for a given perception type index.
 * This function is called when processing the perceptions from a sensor group.
 */
template<std::size_t I>
void perception_handler_fn(
  const std::string & group,
  const std::vector<easynav::PerceptionPtr> & perceptions,
  ::easynav::NavState & ns
)
{
  using P = std::tuple_element_t<I, Registry>;
  ns.set(group, get_perceptions<P>(perceptions));
}

template<std::size_t... Is>
constexpr auto make_handler_table(std::index_sequence<Is...>)
{
  return std::array<SensorsNode::SensorsHandlerFn, sizeof...(Is)>{&perception_handler_fn<Is>...};
}

/// Compile-time helper table that maps type indices from the Registry to handler function pointers
constexpr auto perception_handler_table =
  make_handler_table(std::make_index_sequence<std::tuple_size_v<Registry>>());

}

// SensorsNode member function to populate the runtime map using compile-time template recursion
template<std::size_t I>
void SensorsNode::populate_group_to_handler_map()
{
  if constexpr (I < std::tuple_size_v<Registry>) {
    using P = std::tuple_element_t<I, Registry>;
    group_to_handler_.emplace(std::string(P::default_group_), perception_handler_table[I]);
    populate_group_to_handler_map<I + 1>();
  }
}

bool
SensorsNode::set_by_group(
  const std::string & group,
  const std::vector<easynav::PerceptionPtr> & perceptions,
  ::easynav::NavState & ns)
{
  // Find the perception handler function to process this group
  auto it = group_to_handler_.find(group);
  if (it == group_to_handler_.end()) {
    // If there is no handler registered for this group, return false
    return false;
  }

  // Call the handler function pointer stored for this group
  it->second(group, perceptions, ns);
  return true;
}


SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("sensors_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  percept_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "sensors_node/perceptions", rclcpp::SensorDataQoS());

  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("forget_time")) {
    declare_parameter("forget_time", 1.0);
  }

  ::easynav::NavState::register_printer<easynav::PointPerceptions>(
    [](const easynav::PointPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "PointPerception " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << perception->data.size() << " points in frame [" << perception->frame_id
              << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::ImagePerceptions>(
    [](const easynav::ImagePerceptions & perceptions) {
      std::ostringstream ret;
      ret << "ImagePerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << "Image (" << perception->data.cols << " x  " << perception->data.rows << ")"
              << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::DetectionsPerceptions>(
    [](const easynav::DetectionsPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "DetectionsPerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << " --> "
              << "Detections: " << perception->data.detections.size()
              << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::IMUPerceptions>(
    [](const easynav::IMUPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "IMUPerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << "IMUPerception linear acc = (" <<
          perception->data.linear_acceleration.x << ", " <<
          perception->data.linear_acceleration.y << ", " <<
          perception->data.linear_acceleration.z << ")\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::GNSSPerceptions>(
    [](const easynav::GNSSPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "GNSSPerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        const auto & fix = perception->data;
        ret << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
            << "GNSSPerception lat = " << fix.latitude
            << ", lon = " << fix.longitude
            << ", alt = " << fix.altitude
            << " (status: " << static_cast<int>(fix.status.status)
            << ", service: " << fix.status.service << ")"
            << " in frame [" << perception->frame_id << "]"
            << " with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
    });

  register_handler(std::make_shared<PointPerceptionHandler>());
  register_handler(std::make_shared<ImagePerceptionHandler>());
  register_handler(std::make_shared<IMUPerceptionHandler>());
  register_handler(std::make_shared<GNSSPerceptionHandler>());
  register_handler(std::make_shared<DetectionsPerceptionsHandler>());
  // Populate map from default group strings to handler function pointers for fast dispatch
  populate_group_to_handler_map();
}

SensorsNode::~SensorsNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
CallbackReturnT
SensorsNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, group;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);

    // Resolve canonical group from message type
    const std::string canonical_group = resolve_group_from_msg(msg_type);

    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", canonical_group);
    }

    get_parameter(sensor_id + ".group", group);

    // Find handler for the requested group (may be custom/aliased)
    auto handler_it = handlers_.find(group);
    if (handler_it == handlers_.end() && !canonical_group.empty()) {
      // No handler for custom group; try aliasing to canonical group
      const auto canonical_handler_it = handlers_.find(canonical_group);
      if (canonical_handler_it != handlers_.end()) {
        // Use handler from canonical group
        handlers_[group] = canonical_handler_it->second;
        const auto canonical_fn_it = group_to_handler_.find(canonical_group);
        if (canonical_fn_it != group_to_handler_.end()) {
          group_to_handler_[group] = canonical_fn_it->second;
        }
        handler_it = canonical_handler_it;
        RCLCPP_INFO(get_logger(),
                    "Aliased group '%s' -> '%s' for type '%s'",
                    group.c_str(), canonical_group.c_str(), msg_type.c_str());
      }
    }

    if (handler_it == handlers_.end()) {
      RCLCPP_WARN(get_logger(), "No handler for group [%s]", group.c_str());
      continue;
    }

    const auto perception_ptr = handler_it->second->create(sensor_id);
    const auto sub = handler_it->second->create_subscription(
      *this, topic, msg_type, perception_ptr, realtime_cbg_
    );

    perceptions_[group].emplace_back(PerceptionPtr{perception_ptr, sub});
  }

  return CallbackReturnT::SUCCESS;
}


CallbackReturnT
SensorsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SensorsNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

bool
SensorsNode::cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger)
{
  (void)trigger;

  bool trigger_perceptions = false;

  for (auto & group_perceptions : perceptions_) {
    for (auto & p : group_perceptions.second) {
      trigger_perceptions = trigger_perceptions || p.perception->new_data;
      p.perception->new_data = false;
    }

    if (!set_by_group(group_perceptions.first, group_perceptions.second, *nav_state)) {
      RCLCPP_WARN(get_logger(), "No perception handler for group [%s]",
        group_perceptions.first.c_str());
    }
  }

  return trigger_perceptions;
}

void
SensorsNode::cycle(std::shared_ptr<NavState> nav_state)
{
  for (auto & group_perceptions : perceptions_) {
    for (auto & p : group_perceptions.second) {
      if (p.perception->valid && (now() - p.perception->stamp).seconds() > forget_time_) {
        p.perception->valid = false;
      }
    }
    if (!set_by_group(group_perceptions.first, group_perceptions.second, *nav_state)) {
      RCLCPP_WARN(get_logger(), "No perception handler for group [%s]",
              group_perceptions.first.c_str());
    }
  }

  if (percept_pub_->get_subscription_count() > 0) {
    auto points_perceptions = get_point_perceptions(perceptions_["points"]);

    PointPerceptionsOpsView fused_view(std::move(points_perceptions));

    const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
    const std::string & robot_frame = tf_info.robot_frame;

    fused_view.fuse(robot_frame);
    auto fused_points = fused_view.as_points();

    auto msg = points_to_rosmsg(fused_points);
    msg.header.frame_id = robot_frame;
    const auto & percs = fused_view.get_perceptions();
    if (!percs.empty() && percs[0]) {
      msg.header.stamp = percs[0]->stamp;
    } else {
      msg.header.stamp = now();
    }

    percept_pub_->publish(msg);
  }
}

void
SensorsNode::register_handler(std::shared_ptr<PerceptionHandler> handler)
{
  handlers_[handler->group()] = handler;
}

}  // namespace easynav
