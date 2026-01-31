#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # 用于处理轨迹
import mujoco
import mujoco.viewer
import numpy as np


class MujocoJointPublisher(Node):
    def __init__(self):
        super().__init__('mujoco_joint_publisher')

        # ===== ROS 发布器 =====
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # ===== MuJoCo 模型设置 =====
        xml_path = "/home/an/dm_robot_ws/src/dm_arm_mujoco/mjcf/scene_pick.xml"
        self.get_logger().info(f"加载 MJCF: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 查看 timestep 值
        self.get_logger().info(f"Model timestep: {self.model.opt.timestep}")

        # ===== joints to publish（只包含机械臂的6个关节）=====
        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        ]

        # ===== map: joint name -> qpos/qvel index =====
        self.joint_qposadr = []
        self.joint_dofadr = []
        for jn in self.joint_names:
            j_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if j_id < 0:
                raise RuntimeError(f"joint '{jn}' not found in MuJoCo model")

            qadr = int(self.model.jnt_qposadr[j_id])
            dadr = int(self.model.jnt_dofadr[j_id])

            self.joint_qposadr.append(qadr)
            self.joint_dofadr.append(dadr)

            self.get_logger().info(f"{jn}: joint_id={j_id}, qposadr={qadr}, dofadr={dadr}")

        # ===== timing =====
        self.publish_dt = 0.01  # 50Hz 发布，减慢更新频率
        self.steps_per_tick = max(1, int(round(self.publish_dt / self.model.opt.timestep)))

        # 记录仿真步长
        self.get_logger().info(
            f"publish_dt={self.publish_dt}s, model.timestep={self.model.opt.timestep}s, "
            f"steps_per_tick={self.steps_per_tick}"
        )

        self.timer = self.create_timer(self.publish_dt, self.timer_callback)

        # ===== viewer (可选) =====
        self.enable_viewer = True
        self.viewer = None
        if self.enable_viewer:
            self.viewer = mujoco.viewer.launch_passive(model=self.model, data=self.data)
            self.get_logger().info("MuJoCo viewer launched (passive).")

        # ===== Joint Trajectory Subscription =====
        self.trajectory_subscriber_ = self.create_subscription(JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10)

        # 初始化 current_positions 和 target_positions
        self.current_positions = [0.0] * len(self.joint_names)  # 当前机械臂的关节位置
        self.target_positions = [0.0] * len(self.joint_names)  # 外部控制的目标位置

        self.trajectory_points = []  # 存储接收到的轨迹点
        self.trajectory_index = 0  # 记录当前轨迹点的位置

        self.get_logger().info("Node started.")

    def trajectory_callback(self, msg: JointTrajectory):
        """接收到完整的关节轨迹，更新轨迹目标"""
        self.trajectory_points = msg.points
        self.trajectory_index = 0  # 从轨迹的起始点开始
        self.get_logger().info(f"Received trajectory with {len(msg.points)} points")

    def lerp(self, start, end, alpha):
        """线性插值：逐渐从当前关节位置过渡到目标关节位置"""
        return start + alpha * (end - start)

    def move_to_trajectory(self):
        """根据轨迹点逐步移动机械臂"""
        if self.trajectory_index < len(self.trajectory_points):
            target = self.trajectory_points[self.trajectory_index]

            # 确保轨迹的长度与关节数量匹配
            if len(target.positions) != len(self.joint_names):
                self.get_logger().warning(f"Trajectory size mismatch: Expected {len(self.joint_names)} positions, but got {len(target.positions)}")
                return  # 或者可以采取其他处理措施，例如跳过此轨迹点

            alpha = 1.0  # 更小的步长使得目标位置变化更平缓
            for i, qposadr in enumerate(self.joint_qposadr):
                self.current_positions[i] = self.lerp(self.current_positions[i], target.positions[i], alpha)
                self.data.qpos[qposadr] = self.current_positions[i]

            # 轨迹点移动完，更新到下一个
            if np.allclose(self.current_positions, target.positions, atol=1e-3):
                self.trajectory_index += 1

            # If we've reached the final trajectory point, stop moving
            if self.trajectory_index >= len(self.trajectory_points):
                self.get_logger().info("Reached final position, stopping further movement.")
                self.trajectory_points = []  # Clear trajectory points to stop any further movement

    def timer_callback(self):
        # ===== step multiple times to match real-time =====
        if len(self.trajectory_points) > 0:
            self.move_to_trajectory()  # 根据轨迹更新关节
        else:
            # 如果没有轨迹，保持目标位置
            for i, qposadr in enumerate(self.joint_qposadr):
                self.data.qpos[qposadr] = self.current_positions[i]

        # 执行一次仿真步进
        mujoco.mj_step(self.model, self.data)

        # ===== publish JointState =====
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(self.data.qpos[i]) for i in self.joint_qposadr]
        msg.velocity = [float(self.data.qvel[i]) for i in self.joint_dofadr]
        msg.effort = [0.0] * len(self.joint_names)  # 先占位，后续可接真实力矩
        self.publisher_.publish(msg)

        # ===== viewer sync =====
        if self.viewer is not None:
            self.viewer.sync()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoJointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
