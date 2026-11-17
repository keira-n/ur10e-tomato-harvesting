#!/usr/bin/env python3
"""Receive base-frame tomato points and request a MoveIt plan.

Planning-only is the default. Execution must be explicitly enabled after
validating calibration, TCP, collision geometry, and approach strategy.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint
from moveit_msgs.action import MoveGroup


class TomatoMover(Node):
    def __init__(self):
        super().__init__('tomato_mover')
        self.declare_parameter('target_topic', '/tomato/target_point')
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_link', 'tool0')
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('plan_only', True)
        self.declare_parameter('min_target_separation', 0.05)

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.busy = False
        self.last_target = None
        self.target_topic = self.get_parameter('target_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.subscription = self.create_subscription(
            PointStamped, self.target_topic, self.target_callback, 10
        )
        self.get_logger().info(
            'Listening for targets on %s. plan_only=%s' %
            (self.target_topic, self.get_parameter('plan_only').value)
        )

    def target_callback(self, msg):
        if msg.header.frame_id != self.base_frame:
            self.get_logger().error(
                'Ignoring target in frame %s; expected %s' %
                (msg.header.frame_id, self.base_frame)
            )
            return
        point = (msg.point.x, msg.point.y, msg.point.z)
        if not all(map(lambda v: abs(v) < 10.0, point)):
            self.get_logger().error('Ignoring implausible target coordinates: %s' % (point,))
            return
        if self.busy:
            return
        min_sep = float(self.get_parameter('min_target_separation').value)
        if self.last_target is not None:
            delta = sum((point[i] - self.last_target[i]) ** 2 for i in range(3)) ** 0.5
            if delta < min_sep:
                return
        if not self.client.server_is_ready():
            self.get_logger().warning('MoveIt /move_action is not ready; target not sent')
            return
        self.last_target = point
        self.send_goal(point)

    def send_goal(self, point):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.get_parameter('planning_group').value
        request.allowed_planning_time = float(self.get_parameter('planning_time').value)
        request.num_planning_attempts = 5
        request.max_velocity_scaling_factor = 0.10
        request.max_acceleration_scaling_factor = 0.10

        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = self.get_parameter('tool_link').value
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(self.get_parameter('position_tolerance').value)]
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.pose.position.x = float(point[0])
        target_pose.pose.position.y = float(point[1])
        target_pose.pose.position.z = float(point[2])
        target_pose.pose.orientation.w = 1.0
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(target_pose.pose)
        position.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(position)
        request.goal_constraints.append(constraints)

        plan_only = bool(self.get_parameter('plan_only').value)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False
        self.busy = True
        self.get_logger().info(
            'Requesting %s for %s target: base XYZ=(%.3f, %.3f, %.3f) m' %
            ('plan only' if plan_only else 'PLAN + EXECUTION',
             request.group_name, point[0], point[1], point[2])
        )
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error('MoveIt goal request failed: %s' % exc)
            self.busy = False
            return
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected target')
            self.busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
            code = result.error_code.val
            if code == 1:
                self.get_logger().info('MoveIt reported SUCCESS')
            else:
                self.get_logger().error('MoveIt returned error code %d' % code)
        except Exception as exc:
            self.get_logger().error('MoveIt result failed: %s' % exc)
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = TomatoMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
