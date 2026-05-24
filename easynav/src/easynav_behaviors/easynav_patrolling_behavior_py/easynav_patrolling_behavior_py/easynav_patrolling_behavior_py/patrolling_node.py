# Copyright 2025 Intelligent Robotics Lab
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from enum import Enum

import math
from typing import List, Tuple, Optional 

import rclpy
try:
    from easynav_goalmanager_py import GoalManagerClient, ClientState  # type: ignore
except Exception:  # pragma: no cover
    GoalManagerClient = None
    ClientState = None

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Goals
from tf_transformations import quaternion_from_euler

from builtin_interfaces.msg import Time

class PatrolState(Enum):
    IDLE = 0
    PATROLLING = 1
    FINISHED = 2
    ERROR = 3

def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

class PatrollingNode(Node):
    def __init__(self):
        super().__init__('patrolling_node')

        wp = ['']

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints', wp)

        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value

        wp_names = self.get_parameter('waypoints').get_parameter_value().string_array_value
        
        self._goals = Goals()
        self._goals.header.frame_id = self.frame_id

        for name in wp_names:
            vals = [0.0, 0.0, 0.0]
            self.declare_parameter(name, vals)
            param = self.get_parameter(name)
            if not param:
                self.get_logger().warn(f'Missing waypoint parameter: {name}')
                continue
            vals = param.get_parameter_value().double_array_value
            if len(vals) != 3:
                self.get_logger().warn(f'Waypoint {name} must have 3 values [x, y, yaw]')
                continue

            new_wp = PoseStamped()
            new_wp.header.frame_id = self.frame_id
            new_wp.pose.position.x = vals[0]
            new_wp.pose.position.y = vals[1]

            q = quaternion_from_euler(0, 0, vals[2])

            new_wp.pose.orientation.x = q[0]
            new_wp.pose.orientation.y = q[1]
            new_wp.pose.orientation.z = q[2]
            new_wp.pose.orientation.w = q[3]

            self._goals.goals.append(new_wp)

        self._gm = GoalManagerClient(node=self)

        self._wp_index = 0
        self._state = PatrolState.IDLE

        self._timer = self.create_timer(0.2, self._cycle)
        self.get_logger().info('PatrollingNode started')


    def _cycle(self):
        match self._state:
            case PatrolState.IDLE:
                nav_state = self._gm.get_state()

                if nav_state != ClientState.ACCEPTED_AND_NAVIGATING:
                    self.get_logger().debug('Goals sent')
                    self._goals.header.stamp = self.get_clock().now().to_msg()
                    self._gm.send_goals(self._goals)
                else:
                    self._state = PatrolState.PATROLLING
            case PatrolState.PATROLLING:
                nav_state = self._gm.get_state()
                if (
                    nav_state == ClientState.NAVIGATION_REJECTED or
                    nav_state == ClientState.NAVIGATION_FAILED or
                    nav_state == ClientState.NAVIGATION_CANCELLED or
                    nav_state == ClientState.ERROR
                ):
                    result = self._gm.get_result()
                    self.get_logger().debug(
                        f'Navigation finished with error {result.status_message}')
                    self._state = PatrolState.ERROR
                elif nav_state == ClientState.NAVIGATION_FINISHED:
                    result = self._gm.get_result()
                    self.get_logger().info(
                        f'Navigation succesfully finished with message {result.status_message}')
                    self._state = PatrolState.FINISHED
                elif nav_state == ClientState.ACCEPTED_AND_NAVIGATING:
                    self.get_logger().info('Goals accepted and navigating')
                else:
                    pass
            case PatrolState.FINISHED:
                self.get_logger().info('Reset navigation')
                
                nav_state = self._gm.get_state()
                if nav_state != ClientState.IDLE:
                    self._gm.reset()
                else:
                    self._state = PatrolState.IDLE
            case PatrolState.ERROR:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
