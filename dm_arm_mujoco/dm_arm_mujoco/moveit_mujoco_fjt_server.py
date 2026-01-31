#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory

class MoveItPlanReader(Node):
    def __init__(self):
        super().__init__('moveit_plan_reader')
        self.sub = self.create_subscription(DisplayTrajectory,'/display_planned_path',self.cb,10)
        self.get_logger().info("Subscribed to /display_planned_path")

        # 创建发布器
        self.pub = self.create_publisher(JointTrajectory,'joint_trajectory',10)

    def cb(self, msg: DisplayTrajectory):
        # 检查是否有轨迹数据
        if not msg.trajectory:
            self.get_logger().warn("No trajectory in DisplayTrajectory.")
            return

        self.get_logger().info(f"Received DisplayTrajectory: {len(msg.trajectory)} segment(s)")

        # 通常第一段就是你刚规划出来的那条
        robot_traj = msg.trajectory[0]

        jt = robot_traj.joint_trajectory
        if not jt.joint_names:
            self.get_logger().warn("Empty joint_trajectory.")
            return

        self.get_logger().info(f"Joint names ({len(jt.joint_names)}): {jt.joint_names}")
        self.get_logger().info(f"Points: {len(jt.points)}")

        # 只取最新的轨迹点并发布
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = jt.joint_names
        joint_trajectory.points = jt.points

        # 发布关节轨迹
        self.pub.publish(joint_trajectory)
        self.get_logger().info(f"joint_names {jt.points[0]} ")       
        self.get_logger().info(f"Published joint_trajectory with {len(joint_trajectory.points)} points")

def main():
    rclpy.init()
    node = MoveItPlanReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
