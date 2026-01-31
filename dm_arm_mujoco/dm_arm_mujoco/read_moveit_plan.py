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

    def cb(self, msg: DisplayTrajectory):
        # DisplayTrajectory 里可能包含多段 trajectory
        if not msg.trajectory:
            self.get_logger().warn("No trajectory in DisplayTrajectory.")
            return

        self.get_logger().info(f"Received DisplayTrajectory: {len(msg.trajectory)} segment(s)")

        # 通常第一段就是你刚规划出来的那条
        robot_traj = msg.trajectory[0]

        jt: JointTrajectory = robot_traj.joint_trajectory
        if not jt.joint_names:
            self.get_logger().warn("Empty joint_trajectory.")
            return

        self.get_logger().info(f"Joint names ({len(jt.joint_names)}): {jt.joint_names}")
        self.get_logger().info(f"Points: {len(jt.points)}")

        # 打印前3个点看看数据结构
        show_n = min(3, len(jt.points))
        for k in range(show_n):
            p = jt.points[k]
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            self.get_logger().info(
                f"[pt{k}] t={t:.3f}s "
                f"pos[0]={p.positions[0]:.4f} vel[0]={p.velocities[0] if p.velocities else 'NA'}"
            )


def main():
    rclpy.init()
    node = MoveItPlanReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
