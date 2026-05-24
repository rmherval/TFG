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

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"

class CoreMethodTestCase : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

// Mock class to test the behaviour of MethodBase initialization
class MockMethod : public easynav::MethodBase
{
public:
  MockMethod() = default;
  ~MockMethod() = default;

  void on_initialize() override
  {
    on_initialize_called_ = true;
  }

public:
  bool on_initialize_called_ {false};
};

/** Dummy method class to test behvaiour of derived methods */
class TestLocalizer : public easynav::LocalizerMethodBase
{
public:
  TestLocalizer() = default;
  ~TestLocalizer() = default;

  void on_initialize() override
  {
    odom_.header.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_frame;
    odom_.pose.pose.position.x = 5;
  }

  virtual void update_rt(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

  virtual void update(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

private:
  nav_msgs::msg::Odometry odom_ {};
};


TEST(MethodBaseTest, DefaultConstructor)
{
  easynav::MethodBase method;
  EXPECT_EQ(
    method.get_node(),
    nullptr
  ) << "Default constructor should initialize parent_node_ to nullptr.";
}

TEST_F(CoreMethodTestCase, InitializeSetsParentNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  easynav::MethodBase method;
  method.initialize(node, "test");
}

TEST_F(CoreMethodTestCase, OnInitializeCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  MockMethod method;
  method.initialize(node, "test");
  EXPECT_TRUE(method.on_initialize_called_) <<
    "on_initialize() should be called during initialization.";
}

TEST_F(CoreMethodTestCase, TFInfoPropagatesToDerived)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_tfinfo_node");

  class TFInfoProbeMethod : public easynav::MethodBase
  {
public:
    void on_initialize() override
    {
      seen_tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
    }

    easynav::TFInfo seen_tf_info;
  };

  TFInfoProbeMethod method;
  easynav::TFInfo tf_info;
  tf_info.tf_prefix = "robot_1";
  tf_info.map_frame = "my_map";
  tf_info.odom_frame = "my_odom";
  tf_info.robot_frame = "my_base";
  tf_info.world_frame = "my_world";

  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  method.initialize(node, "test_plugin");

  EXPECT_EQ(method.seen_tf_info.tf_prefix, "robot_1");
  EXPECT_EQ(method.seen_tf_info.map_frame, "robot_1/my_map");
  EXPECT_EQ(method.seen_tf_info.odom_frame, "robot_1/my_odom");
  EXPECT_EQ(method.seen_tf_info.robot_frame, "robot_1/my_base");
  EXPECT_EQ(method.seen_tf_info.world_frame, "robot_1/my_world");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
