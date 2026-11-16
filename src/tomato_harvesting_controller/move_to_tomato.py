#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint
from moveit_msgs.action import MoveGroup


class TomatoMover(Node):

    def __init__(self):
        super().__init__('tomato_mover')

        self.client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info(
            'Waiting for MoveIt action server...'
        )

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'MoveIt action server not available!'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            'MoveIt action server connected!'
        )

        self.send_goal()

    def send_goal(self):

        goal = MoveGroup.Goal()

        # ==========================================
        # MoveIt planning configuration
        # ==========================================

        goal.request.group_name = 'ur_manipulator'

        # Give OMPL more time
        goal.request.allowed_planning_time = 15.0

        # More planning attempts
        goal.request.num_planning_attempts = 20

        # Slow and safe
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10

        # ==========================================
        # Target
        #
        # Red tomato:
        #   (0.97, 0.03, 1.08)
        #
        # Safe approach:
        #   (0.97, 0.03, 1.25)
        # ==========================================

        position = PositionConstraint()

        position.header.frame_id = 'base_link'
        position.link_name = 'tool0'

        # Allow tool0 to reach within 3 cm
        # of the target.
        sphere = SolidPrimitive()

        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]

        position.constraint_region.primitives.append(
            sphere
        )

        target_pose = PoseStamped()

        target_pose.header.frame_id = 'base_link'

        target_pose.pose.position.x = 0.97
        target_pose.pose.position.y = 0.03
        target_pose.pose.position.z = 1.25

        target_pose.pose.orientation.w = 1.0

        position.constraint_region.primitive_poses.append(
            target_pose.pose
        )

        position.weight = 1.0

        # ==========================================
        # Constraints
        #
        # POSITION ONLY
        #
        # We deliberately do not constrain the
        # orientation for this first test.
        # ==========================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position
        )

        goal.request.goal_constraints.append(
            constraints
        )

        # ==========================================
        # Execute
        # ==========================================

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Sending UR10 target to MoveIt'
        )

        self.get_logger().info(
            'Planning group: ur_manipulator'
        )

        self.get_logger().info(
            'Tool link: tool0'
        )

        self.get_logger().info(
            'Frame: base_link'
        )

        self.get_logger().info(
            'Target: X=0.97 Y=0.03 Z=1.25'
        )

        self.get_logger().info(
            'Planning time: 15 seconds'
        )

        self.get_logger().info(
            '========================================'
        )

        future = self.client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt REJECTED the goal!'
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            'MoveIt ACCEPTED the goal!'
        )

        self.get_logger().info(
            'Waiting for planning/execution...'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result

        code = result.error_code.val

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'MoveIt error code: {code}'
        )

        if code == 1:

            self.get_logger().info(
                'SUCCESS!'
            )

            self.get_logger().info(
                'UR10 should now be near the '
                'tomato approach position.'
            )

        elif code == -1:

            self.get_logger().error(
                'PLANNING_FAILED'
            )

        elif code == -31:

            self.get_logger().error(
                'NO_IK_SOLUTION'
            )

        elif code == -12:

            self.get_logger().error(
                'GOAL_IN_COLLISION'
            )

        elif code == -6:

            self.get_logger().error(
                'TIMED_OUT'
            )

        else:

            self.get_logger().error(
                f'MoveIt returned FAILURE code {code}'
            )

        self.get_logger().info(
            '========================================'
        )

        rclpy.shutdown()


def main():

    rclpy.init()

    node = TomatoMover()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
