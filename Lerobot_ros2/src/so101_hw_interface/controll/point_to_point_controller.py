#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from typing import List, Dict
import time


class PointToPointController(Node):
    def __init__(self):
        super().__init__('point_to_point_controller')
        self.joint_pub = self.create_publisher(JointState, 'so101_follower/joint_commands', 10)

        # ジョイント名の定義
        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        # 各ジョイントの現在位置（ラジアン）
        self.current_positions = {name: 0.0 for name in self.joint_names}
        
        # 目標位置と移動時間のリスト
        self.target_positions = {name: 0.0 for name in self.joint_names}
        self.start_positions = {name: 0.0 for name in self.joint_names}
        
        # 移動制御用パラメータ
        self.is_moving = False
        self.move_start_time = 0.0
        self.move_duration = 0.0  # 移動にかける時間（秒）
        
        # ウェイポイントのキュー
        self.waypoints = []  # [(positions_dict, duration), ...]
        self.current_waypoint_index = 0
        
        # 制御周期（20Hz）
        self.control_rate = 20  # Hz
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_callback)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Point-to-Point Controller started!")
        self.get_logger().info("=" * 60)

    def add_waypoint(self, positions: Dict[str, float], duration: float):
        """
        ウェイポイントを追加
        
        Args:
            positions: ジョイント名と目標角度（ラジアン）の辞書
            duration: この位置への移動時間（秒）
        """
        # 前のウェイポイントの最終位置を基準にする
        if self.waypoints:
            # 前のウェイポイントが存在する場合、その位置をコピー
            full_positions = self.waypoints[-1][0].copy()
        else:
            # 最初のウェイポイントの場合は現在位置を基準にする
            full_positions = {name: 0.0 for name in self.joint_names}
        
        # 指定されたジョイントのみ更新
        full_positions.update(positions)
        
        self.waypoints.append((full_positions, duration))
        self.get_logger().info(f"Waypoint added (total: {len(self.waypoints)})")
        self.get_logger().info(f"  Duration: {duration:.2f}s")
        for name, pos in positions.items():
            deg = math.degrees(pos)
            self.get_logger().info(f"  {name:15s}: {pos:+.3f} rad ({deg:+.1f}°)")

    def start_execution(self):
        """ウェイポイントの実行を開始"""
        if not self.waypoints:
            self.get_logger().warning("No waypoints to execute!")
            return
        
        self.current_waypoint_index = 0
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting waypoint execution...")
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
        self.get_logger().info("=" * 60)
        self._start_next_waypoint()

    def _start_next_waypoint(self):
        """次のウェイポイントへの移動を開始"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("=" * 60)
            self.get_logger().info("All waypoints completed!")
            self.get_logger().info("=" * 60)
            self.is_moving = False
            return
        
        target_pos, duration = self.waypoints[self.current_waypoint_index]
        
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        self.get_logger().info(f"Duration: {duration:.2f}s")
        
        # 開始位置を保存
        self.start_positions = self.current_positions.copy()
        self.target_positions = target_pos.copy()
        self.move_duration = duration
        self.move_start_time = time.time()
        self.is_moving = True
        
        # 移動の詳細をログ出力
        for name in self.joint_names:
            start_deg = math.degrees(self.start_positions[name])
            target_deg = math.degrees(self.target_positions[name])
            delta = self.target_positions[name] - self.start_positions[name]
            delta_deg = math.degrees(delta)
            self.get_logger().info(
                f"  {name:15s}: {self.start_positions[name]:+.3f} -> {self.target_positions[name]:+.3f} "
                f"({start_deg:+.1f}° -> {target_deg:+.1f}°, delta: {delta_deg:+.1f}°)"
            )
        self.get_logger().info("-" * 60)

    def control_callback(self):
        """制御ループのコールバック"""
        if not self.is_moving:
            # 移動中でない場合は現在位置を維持
            self.publish_joint_state()
            return
        
        # 経過時間を計算
        elapsed_time = time.time() - self.move_start_time
        
        if elapsed_time >= self.move_duration:
            # 移動完了: 目標位置に到達
            self.current_positions = self.target_positions.copy()
            self.publish_joint_state()
            
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached!")
            
            # 次のウェイポイントへ
            self.current_waypoint_index += 1
            self._start_next_waypoint()
        else:
            # 移動中: 線形補間で中間位置を計算
            progress = elapsed_time / self.move_duration
            
            for name in self.joint_names:
                start = self.start_positions[name]
                target = self.target_positions[name]
                self.current_positions[name] = start + (target - start) * progress
            
            self.publish_joint_state()

    def publish_joint_state(self):
        """現在のジョイント状態を発行"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.current_positions[n] for n in self.joint_names]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = PointToPointController()
    
    # サンプルウェイポイントの追加
    node.get_logger().info("Adding sample waypoints...")
    
    # ウェイポイント1: initial position (3.0)
    node.add_waypoint({
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.5,  # グリッパーを開く
    }, duration=1.0)
    
    # ウェイポイント2: ready to pick position (3.0)
    node.add_waypoint({
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.280718,
        "elbow_flex": -0.081300,
        "wrist_flex": 1.2946797,
        "wrist_roll": 0.0138058,
    }, duration=1.0)

    node.add_waypoint({
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.556835,
        "elbow_flex": -0.19481556,
        "wrist_flex": 1.285475,
        "wrist_roll": 0.0138058,
    }, duration=1.0)
    
    # ウェイポイント3: elbowを曲げる（2秒）
    node.add_waypoint({
        "gripper": -0.01,  # gripper close
    }, duration=1.0)
    
    # ウェイポイント4: next position
    node.add_waypoint({
        "shoulder_pan": -1.27933,
        "shoulder_lift": -0.8038,
        "elbow_flex": 0.28685,
        "wrist_flex": 0.82528,
        "wrist_roll": 0.0076699,
    }, duration=1.0)
    
    # ウェイポイント5:
    node.add_waypoint({
        "shoulder_pan": -1.133611,
        "shoulder_lift": 0.314466,
        "elbow_flex": 0.079767,
        "wrist_flex": 1.13514,
        "wrist_roll": -0.010737,
    }, duration=1.0)

    node.add_waypoint({
        "gripper": 0.5,  # グリッパーを開く
    }, duration=1.0)

    node.add_waypoint({
        "shoulder_pan": -1.133611,
        "shoulder_lift": 0.0582912,
        "elbow_flex": 0.0260776,
        "wrist_flex": 1.3437671,
        "wrist_roll": -0.010737,
    }, duration=1.0)

    node.add_waypoint({
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.5,  # グリッパーを開く
    }, duration=1.0)
    
    # 実行開始
    node.start_execution()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
