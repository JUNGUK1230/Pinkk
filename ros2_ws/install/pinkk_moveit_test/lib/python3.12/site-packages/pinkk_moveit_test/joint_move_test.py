import rclpy
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient


class JointMoveTest(Node):

    def __init__(self):
        super().__init__('joint_move_test')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Waiting controller...")
        self.client.wait_for_server()

        self.send_goal()


    def send_goal(self):

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6"
        ]

        point = JointTrajectoryPoint()

        # 테스트용 작은 움직임
        point.positions = [
            0.2,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        point.time_from_start.sec = 3

        goal.trajectory.points.append(point)

        self.client.send_goal_async(goal)


def main():

    rclpy.init()

    node = JointMoveTest()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
