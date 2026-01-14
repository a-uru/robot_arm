#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math


class PS4ArmBridge(Node):
    def __init__(self):
        super().__init__('ps4_arm_bridge')
        self.joint_pub = self.create_publisher(JointState, 'so101_follower/joint_commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

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

        # PS4コントローラーの軸マッピング
        # 軸0: 左スティック左右, 軸1: 左スティック上下
        # 軸2: 右スティック上下, 軸3: 右スティック左右
        self.axis_map = {
            6: "shoulder_pan",    # 十字ボタン左右
            7: "shoulder_lift",   # 十字ボタン上下
        }
        
        # ボタンマッピング
        self.button_map = {
            2: "elbow_flex",       # △ボタン
            0: "elbow_flex",       # ☓ボタン
            3: "wrist_flex",       # □ボタン
            1: "wrist_flex",       # ○ボタン
            4: "wrist_roll",       # L1
            5: "wrist_roll",       # R1
            10: "gripper",         # PSボタン
        }

        # 制御パラメータ
        self.scale = 0.015  # 軸の入力値を角度増分(rad)に変換するスケール
        self.button_step = 0.03  # ボタン押下時の角度増分（固定）
        self.deadzone = 0.1  # デッドゾーン（スティックの遊び）

        # デバッグ用フラグとカウンター
        self.show_periodic_status = True  # 定期的な状態表示
        self.publish_counter = 0
        self.status_interval = 100  # 100回に1回状態を表示（約5秒ごと）
        
        # 最後に受信したJoyメッセージのタイムスタンプ
        self.last_joy_time = None
        
        # チャタリング防止用: 各ボタンの最後の押下時刻と状態
        self.last_button_press_time = {}
        self.last_button_state = {}  # 前回のボタン状態を記録
        self.button_repeat_time = 0.05  # ボタンを押し続けた時の繰り返し間隔（秒）

        # 定期的にコマンドを送信（20Hz）
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("PS4 Arm Bridge started!")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Control mapping:")
        self.get_logger().info("  D-pad Left/Right: shoulder_pan")
        self.get_logger().info("  D-pad Up/Down:    shoulder_lift")
        self.get_logger().info("  △ / ☓ buttons:   elbow_flex")
        self.get_logger().info("  □ / ○ buttons:   wrist_flex")
        self.get_logger().info("  L1 / R1:          wrist_roll")
        self.get_logger().info("  PS button:        gripper (toggle)")
        self.get_logger().info("=" * 60)

    def apply_deadzone(self, value):
        """デッドゾーンを適用"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def can_press_button(self, button_id, is_pressed, allow_repeat=True):
        """ボタンが押下可能かチェック
        
        Args:
            button_id: ボタンID
            is_pressed: ボタンが押されているか
            allow_repeat: 押し続けた時にリピートを許可するか（False=エッジ検出のみ）
        """
        current_time = self.get_clock().now()
        
        # ボタンが押されていない場合
        if not is_pressed:
            self.last_button_state[button_id] = False
            return False
        
        # 前回の状態を取得（初回はFalse）
        was_pressed = self.last_button_state.get(button_id, False)
        
        # ボタンが新しく押された場合（エッジ）
        if not was_pressed:
            self.last_button_state[button_id] = True
            self.last_button_press_time[button_id] = current_time
            return True
        
        # リピートが許可されていない場合は、ここで終了（エッジ検出のみ）
        if not allow_repeat:
            return False
        
        # ボタンが押し続けられている場合（リピート許可時のみ）
        # 最後の実行から一定時間経過していればOK
        if button_id in self.last_button_press_time:
            time_since_last = (current_time - self.last_button_press_time[button_id]).nanoseconds / 1e9
            if time_since_last >= self.button_repeat_time:
                self.last_button_press_time[button_id] = current_time
                return True
        
        return False

    def joy_cb(self, msg: Joy):
        """ジョイスティック入力のコールバック"""
        # Joyメッセージ受信時刻を記録
        self.last_joy_time = self.get_clock().now()
        
        # デバッグ: 受信したJoyメッセージの情報
        self.get_logger().debug(f"Joy received - axes: {len(msg.axes)}, buttons: {len(msg.buttons)}")
        
        # 変更があったかどうかを追跡
        changed = False
        
        # 軸の入力に応じて角度を更新（十字ボタン）
        for idx, joint in self.axis_map.items():
            if idx < len(msg.axes):
                # デッドゾーンを適用した入力値
                axis_value = self.apply_deadzone(msg.axes[idx])
                if axis_value != 0.0:
                    # 角度の増分を計算して加算
                    delta = - axis_value * self.scale
                    
                    # 十字ボタン左右（軸6）は符号を反転
                    if idx == 6:
                        delta = -delta
                    
                    old_pos = self.joint_positions[joint]
                    self.joint_positions[joint] += delta
                    changed = True
                    self.get_logger().info(
                        f"[AXIS {idx}] {joint}: {old_pos:.3f} -> {self.joint_positions[joint]:.3f} "
                        f"(delta: {delta:+.3f}, raw: {msg.axes[idx]:.2f})"
                    )

        # ボタン押下による各ジョイント操作（固定増分方式）
        if len(msg.buttons) > 10:
            # △ボタン(2) - elbow_flex を上げる
            if self.can_press_button(2, msg.buttons[2]):
                old_pos = self.joint_positions["elbow_flex"]
                self.joint_positions["elbow_flex"] -= self.button_step
                changed = True
                self.get_logger().info(
                    f"[△ BTN] elbow_flex: {old_pos:.3f} -> {self.joint_positions['elbow_flex']:.3f} (-{self.button_step})"
                )
            
            # ☓ボタン(0) - elbow_flex を下げる
            if self.can_press_button(0, msg.buttons[0]):
                old_pos = self.joint_positions["elbow_flex"]
                self.joint_positions["elbow_flex"] += self.button_step
                changed = True
                self.get_logger().info(
                    f"[☓ BTN] elbow_flex: {old_pos:.3f} -> {self.joint_positions['elbow_flex']:.3f} (+{self.button_step})"
                )
            
            # □ボタン(3) - wrist_flex を上げる
            if self.can_press_button(3, msg.buttons[3]):
                old_pos = self.joint_positions["wrist_flex"]
                self.joint_positions["wrist_flex"] += self.button_step
                changed = True
                self.get_logger().info(
                    f"[□ BTN] wrist_flex: {old_pos:.3f} -> {self.joint_positions['wrist_flex']:.3f} (+{self.button_step})"
                )
            
            # ○ボタン(1) - wrist_flex を下げる
            if self.can_press_button(1, msg.buttons[1]):
                old_pos = self.joint_positions["wrist_flex"]
                self.joint_positions["wrist_flex"] -= self.button_step
                changed = True
                self.get_logger().info(
                    f"[○ BTN] wrist_flex: {old_pos:.3f} -> {self.joint_positions['wrist_flex']:.3f} (-{self.button_step})"
                )
            
            # L1ボタン(4) - wrist_roll を左回転
            if self.can_press_button(4, msg.buttons[4]):
                old_pos = self.joint_positions["wrist_roll"]
                self.joint_positions["wrist_roll"] -= self.button_step
                changed = True
                self.get_logger().info(
                    f"[L1 BTN] wrist_roll: {old_pos:.3f} -> {self.joint_positions['wrist_roll']:.3f} (-{self.button_step})"
                )
            
            # R1ボタン(5) - wrist_roll を右回転
            if self.can_press_button(5, msg.buttons[5]):
                old_pos = self.joint_positions["wrist_roll"]
                self.joint_positions["wrist_roll"] += self.button_step
                changed = True
                self.get_logger().info(
                    f"[R1 BTN] wrist_roll: {old_pos:.3f} -> {self.joint_positions['wrist_roll']:.3f} (+{self.button_step})"
                )
            
            # PSボタン(10) - gripper の開閉切り替え（リピートなし=エッジ検出のみ）
            if self.can_press_button(10, msg.buttons[10], allow_repeat=False):
                old_pos = self.joint_positions["gripper"]
                # 現在の値に応じて開閉を切り替え
                if self.joint_positions["gripper"] < 0:
                    self.joint_positions["gripper"] = 0.5  # 開く
                    action = "OPEN"
                else:
                    self.joint_positions["gripper"] = -0.5  # 閉じる
                    action = "CLOSE"
                changed = True
                self.get_logger().info(
                    f"[PS BTN] gripper: {old_pos:.3f} -> {self.joint_positions['gripper']:.3f} ({action})"
                )
        
        # 変更があった場合は全ジョイントの状態を表示
        if changed:
            self.get_logger().info("-" * 60)
            self.get_logger().info("Current joint positions:")
            for name in self.joint_names:
                deg = math.degrees(self.joint_positions[name])
                self.get_logger().info(f"  {name:15s}: {self.joint_positions[name]:+.3f} rad ({deg:+.1f}°)")
            self.get_logger().info("-" * 60)

    def publish_joint_state(self):
        """定期的にジョイント状態を発行"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.joint_positions[n] for n in self.joint_names]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)
        
        # 定期的な状態表示（約5秒ごと）
        self.publish_counter += 1
        if self.show_periodic_status and self.publish_counter >= self.status_interval:
            self.publish_counter = 0
            
            # Joyメッセージの受信状態をチェック
            if self.last_joy_time is not None:
                time_since_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
                joy_status = f"OK (last: {time_since_joy:.1f}s ago)"
            else:
                joy_status = "NO JOY MESSAGE RECEIVED"
            
            self.get_logger().info("╔" + "=" * 58 + "╗")
            self.get_logger().info(f"║ PERIODIC STATUS (joy: {joy_status:30s}) ║")
            self.get_logger().info("╠" + "=" * 58 + "╣")
            for name in self.joint_names:
                rad = self.joint_positions[name]
                deg = math.degrees(rad)
                bar_length = 20
                # -π to π の範囲でバーを表示
                if name == "gripper":
                    # gripper は -0.5 to 0.5
                    normalized = (rad + 0.5) / 1.0
                else:
                    # その他は -π to π
                    normalized = (rad + math.pi) / (2 * math.pi)
                bar_pos = int(normalized * bar_length)
                bar_pos = max(0, min(bar_length - 1, bar_pos))
                bar = "░" * bar_pos + "█" + "░" * (bar_length - bar_pos - 1)
                self.get_logger().info(
                    f"║ {name:15s} {rad:+.3f}rad {deg:+6.1f}° [{bar}] ║"
                )
            self.get_logger().info("╚" + "=" * 58 + "╝")

def main():
    rclpy.init()
    node = PS4ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
