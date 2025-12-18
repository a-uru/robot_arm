#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select
import math


class KeyboardArmBridge(Node):
    def __init__(self):
        super().__init__('keyboard_arm_bridge')
        self.joint_pub = self.create_publisher(JointState, 'so101_follower/joint_commands', 10)

        # 各ジョイントの現在位置（ラジアン）
        self.joint_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.joint_names = list(self.joint_positions.keys())

        # キーボードマッピング
        self.key_map = {
            # shoulder_pan (A/D)
            'a': ("shoulder_pan", -0.03),
            'd': ("shoulder_pan", +0.03),
            # shoulder_lift (W/S)
            'w': ("shoulder_lift", -0.03),
            's': ("shoulder_lift", +0.03),
            # elbow_flex (I/K)
            'i': ("elbow_flex", -0.03),
            'k': ("elbow_flex", +0.03),
            # wrist_flex (J/L)
            'j': ("wrist_flex", +0.03),
            'l': ("wrist_flex", -0.03),
            # wrist_roll (U/O)
            'u': ("wrist_roll", +0.03),
            'o': ("wrist_roll", -0.03),
            # gripper (Space)
            ' ': ("gripper", "toggle"),
        }

        # 定期的にコマンドを送信（20Hz）
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        
        # ターミナル設定の保存
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Keyboard Arm Bridge started!")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Control mapping:")
        self.get_logger().info("  W/S:         shoulder_lift (up/down)")
        self.get_logger().info("  A/D:         shoulder_pan (left/right)")
        self.get_logger().info("  I/K:         elbow_flex (up/down)")
        self.get_logger().info("  J/L:         wrist_flex (left/right)")
        self.get_logger().info("  U/O:         wrist_roll (left/right)")
        self.get_logger().info("  Space:       gripper (toggle open/close)")
        self.get_logger().info("  Q:           Quit")
        self.get_logger().info("=" * 60)

    def get_key(self):
        """キーボード入力を取得（ノンブロッキング）"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self, key):
        """キー入力を処理してジョイント位置を更新"""
        if key == 'q' or key == 'Q':
            self.get_logger().info("Quit command received. Shutting down...")
            rclpy.shutdown()
            return False
        
        if key in self.key_map:
            joint, value = self.key_map[key]
            old_pos = self.joint_positions[joint]
            
            if value == "toggle":
                # gripper の開閉切り替え
                if self.joint_positions[joint] < 0:
                    self.joint_positions[joint] = 0.5  # 開く
                    action = "OPEN"
                else:
                    self.joint_positions[joint] = -0.01  # 閉じる
                    action = "CLOSE"
                self.get_logger().info(
                    f"[{key.upper()}] {joint}: {old_pos:.3f} -> {self.joint_positions[joint]:.3f} ({action})"
                )
            else:
                # 通常のジョイント増減
                self.joint_positions[joint] += value
                self.get_logger().info(
                    f"[{key.upper()}] {joint}: {old_pos:.3f} -> {self.joint_positions[joint]:.3f} "
                    f"(delta: {value:+.3f})"
                )
            
            # 現在の全ジョイント状態を表示
            self.get_logger().info("-" * 60)
            self.get_logger().info("Current joint positions:")
            for name in self.joint_names:
                deg = math.degrees(self.joint_positions[name])
                self.get_logger().info(f"  {name:15s}: {self.joint_positions[name]:+.3f} rad ({deg:+.1f}°)")
            self.get_logger().info("-" * 60)
        
        return True

    def publish_joint_state(self):
        """定期的にジョイント状態を発行"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.joint_positions[n] for n in self.joint_names]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)

    def run(self):
        """メインループ"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def destroy_node(self):
        """ノード終了時にターミナル設定を復元"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardArmBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
