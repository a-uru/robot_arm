# MoveIt Servo による3次元位置追従

更新される `geometry_msgs/msg/PointStamped` を受け取り、手先を目標位置へ近づけます。SO-101 は5軸なので、手先の姿勢は拘束しません。既定の制御点は `gripper_link` の原点で、物理的な指先先端とは異なります。

## 構成

```text
/so101_servo/target_point (PointStamped)
  → point_tracker: 現在位置との誤差・位置ヤコビアン・減衰最小二乗法
  → /servo_node/delta_joint_cmds (JointJog)
  → MoveIt Servo: 衝突チェック・関節限界・速度制限・平滑化
  → /arm_controller/joint_trajectory
  → ros2_control → 既存モーターブリッジ → SO-101
```

位置だけを制御するため、6次元の姿勢拘束を伴う Twist ではなく、3×5の位置ヤコビアンで求めた関節速度を Servo の JointJog モードへ渡します。位置ヤコビアンの特異点近傍は減衰最小二乗法で速度を抑え、ランクが落ちても動かせる方向への追従は継続します。局所的に動かせない方向だけを要求した場合はゼロ速度になります。Servo による6次元Cartesian特異点判定はこのモードでは使わず、衝突と関節限界のチェックは有効です。

## インストール・ビルド

```bash
sudo apt-get install -y ros-jazzy-moveit-servo
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --build-base build/moveit --packages-select so101_servo so101_follower_moveit --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

既存のSO-101パッケージが未ビルドなら、上記の `--packages-select` に `so101_follower_description so101_hw_interface` も追加してください。

## 速度・指先位置の設定ファイル

[servo_tracking.yaml](../Lerobot_ros2/src/so101_follower_moveit/config/servo_tracking.yaml) に、指先オフセット・最高速度・加速度・ゲイン・目標タイムアウトをまとめています。通常はこのYAMLを編集してから、短いコマンドで起動してください。

```bash
ros2 launch so101_follower_moveit servo.launch.py use_fake_hardware:=false interactive_target:=true
```

`tip_offset_x/y/z` は現在のtip_frameからの相対位置で、m単位です。実機の指先寸法に合わせて記入します。既定はすべて0.0です。起動時に追従ノードと操作マーカーの両方へ同じ値が渡されます。

別の設定ファイルを使用する場合：

```bash
ros2 launch so101_follower_moveit servo.launch.py tracking_config:=/絶対パス/servo_tracking.yaml
```

以前の `tip_offset_z:=...` や `max_linear_speed:=...` などの起動引数も使用できます。優先順位は **明示した起動引数 → YAML** です。設定変更後は再起動してください。現在の `--symlink-install` の構成では、このYAMLの値を変更しただけなら再ビルドは不要です。

## 起動

既存の `demo.launch.py`、遠隔操作ノード、同じコントローラーへの指令元を停止してから起動します。

模擬アーム：

```bash
ros2 launch so101_follower_moveit servo.launch.py
```

実機：

```bash
ros2 launch so101_follower_moveit servo.launch.py use_fake_hardware:=false port:=/dev/ttyACM0
```

実機モードは最初の位置情報を受信してからコントローラーとServoを起動します。目標位置が来るまでは追従動作を開始しません。通常のMoveGroupは起動しないため、このモードでRVizのPlan/Executeは使いません。RVizには現在のロボットモデルを表示します。

## 目標位置の配信

トピック：`/so101_servo/target_point`
型：`geometry_msgs/msg/PointStamped`
単位：m

既定では、目標を10〜50 Hz程度で更新してください。最後の受信から0.3秒が経過すると、追従ノードはゼロ速度を送り、Servo経由で停止します。単発の目標を永久に保持して追従する仕様ではありません。

模擬アームの初期姿勢付近を目標にする例：

```bash
ros2 topic pub --rate 20 /so101_servo/target_point geometry_msgs/msg/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.03, y: -0.27, z: 0.27}}'
```

実機では現在位置と到達可能範囲に合わせて座標を選んでください。現在の制御点の位置は次で確認できます。

```bash
ros2 run tf2_ros tf2_echo base_link gripper_link
```

Pythonでの配信例：

```python
from geometry_msgs.msg import PointStamped

# node は既存の rclpy.Node。pub は一度作り、タイマーやセンサーコールバックで publish する。
pub = node.create_publisher(PointStamped, '/so101_servo/target_point', 10)
msg = PointStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'
msg.point.x, msg.point.y, msg.point.z = x, y, z
pub.publish(msg)
```

`header.frame_id` は必須です。別の座標系（例：カメラ）でも、`base_link` への正しいTFが存在すれば変換します。時刻を指定した目標にはその時刻のTFを使用します。時刻ゼロは最新TF・受信時刻基準を意味します。時刻を付ける場合は目標配信側とROSノードの時計を揃えてください。古い目標、不正数値、未取得または古い動的TFでは停止します。静的TFは時刻ゼロで使用できます。

グリッパーより先に制御点を置く場合は、URDFに `gripper_link` 配下の固定リンクを追加し、`tip_frame:=追加したリンク名` を指定します。可動する `jaw_link` は制御点にできません。

## 速度・停止条件

既定の追従パラメーター：

| パラメーター | 既定値 | 意味 |
| --- | --- | --- |
| `publish_rate` | 50 Hz | 関節速度指令の周期 |
| `max_linear_speed` | 0.08 m/s | 手先の要求速度上限 |
| `max_joint_speed` | 0.8 rad/s | 関節速度指令上限 |
| `max_joint_acceleration` | 1.0 rad/s² | 通常追従時の関節速度の変化制限 |
| `position_tolerance` | 0.003 m | 位置追従の目標許容距離 |
| `target_timeout` | 0.3秒 | 目標更新がない場合の停止判定 |
| `state_timeout` | 0.2秒 | 関節情報が古い場合の停止判定 |

`servo.launch.py` では速度、加速度、位置誤差ゲイン `gain`（既定4.0）、目標タイムアウト、制御点オフセットを引数で変更できます。Servo自体も `joint_limits.yaml` の制限を使います。RVizのVelocity/Acceleration ScalingはServoの追従速度を設定する項目ではありません。

```bash
ros2 launch so101_follower_moveit servo.launch.py max_linear_speed:=0.02 target_timeout:=0.3
```

Servoへの速度指令が0.1秒途絶えるとServo側のタイムアウトも働きます。ウォッチドッグは指令を止める機能で、実機の停止時間や停止距離を保証するものではありません。

Servo専用の `servo_controllers.yaml` では、継続する軌道の終端速度がゼロでないことを許可し、前回の指令から補間します。通常のPlan/Execute用のコントローラー設定は別のままです。別プロセスから同じコントローラーへ軌道を送らないでください。

## 状態確認

```bash
ros2 topic echo /so101_servo/tracker_status
ros2 topic echo /so101_servo/position_error
ros2 topic echo /servo_node/status
```

- `tracking`：有効な位置情報と目標に基づき関節速度を要求中。Servoが実際に動作を許可したかは `/servo_node/status` も確認します。
- `at_target`：位置誤差が許容距離内。
- `waiting_for_target`：目標なし、または目標更新のタイムアウト。
- `stale_joint_state`：現在の関節角度を取得できていない。
- `tracking_near_singularity`：位置ヤコビアンのランクが不足しているものの、動かせる方向に制限付きで追従中。
- `unreachable_direction`：現在の姿勢から目標方向へ進む関節速度を求められない状態。マーカーを別方向へ動かすと復帰できる場合があります。
- `target_transform_or_kinematics_unavailable`：TFまたは運動学の入力が無効。

Servoは障害物を迂回する経路探索器ではありません。到達不能な目標、衝突、関節限界では減速・停止し、目標へ到達しないことがあります。実機で既に見られた追従誤差やオーバーシュートを、この実装だけで解消するものではありません。

## 検証

運動学の有限差分ヤコビアン比較、速度制限、不正入力をユニットテストで検証します。

```bash
python3 -m pytest Lerobot_ros2/src/so101_servo/test -q
```

実機への目標配信は開発中に行っていません。2026年10月30日に、分離したROS領域の模擬アームで次を確認しました。

- ユニットテスト：5件成功。
- 2つの静止目標：それぞれ位置誤差約2.5 mmに収束。
- 周期4秒、振幅6 mm・4 mmの連続移動目標：8秒間の試験で平均誤差約4.0 mm、最大約6.9 mm。
- 回転0.6 rad・並進を含む静的TFの目標：誤差約2.5 mmに収束。
- 目標更新停止：ゼロ関節速度を確認し、停止後0.7秒の制御点移動量が2 mm未満。
- NaN座標・存在しないTF：ゼロ関節速度を確認。
- 模擬の関節状態ブロードキャスター停止：`stale_joint_state` とゼロ関節速度を確認。

これらは指定した模擬条件での結果で、実機の追従精度・停止距離・全姿勢での到達性を保証しません。

## 参照

- [MoveIt Servo Jazzyパラメーター](https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/config/servo_parameters.yaml)

## RViz のマウス操作で目標を指定する

`interactive_target:=true` を追加すると、ドラッグ可能な緑色の球とXYZ軸の矢印が表示されます。

模擬アーム：

```bash
ros2 launch so101_follower_moveit servo.launch.py interactive_target:=true
```

実機：

```bash
ros2 launch so101_follower_moveit servo.launch.py use_fake_hardware:=false port:=/dev/ttyACM0 interactive_target:=true
```

1. RViz の上部ツールバーで **Interact**（操作ツール）を選びます。
2. マーカーは現在の `gripper_link` の位置に現れます。起動しただけでは目標を配信しません。
3. XYZ軸の矢印をドラッグするとその軸方向に、球をドラッグすると画面上の平面方向に目標を移動できます。
4. ドラッグするとその位置へ追従を開始します。マウスを離しても選んだ位置を30 Hzで配信し続けます。
5. マーカーの右クリックメニュー **Stop tracking / reset to current tool** で配信を止め、現在の手先位置へマーカーを戻します。追従側は目標タイムアウト（既定0.3秒）で停止指令を出します。

マーカーが到達範囲外や障害物の中へ移動しても、Servoは関節限界・衝突チェックを維持します。マーカー位置が必ず実現されるわけではありません。このモード中は、外部から同じ目標トピックへ配信しないでください。別座標系のセンサー目標を使うときは `interactive_target:=false`（既定）で起動してください。

模擬環境で、起動時の目標配信なし、ドラッグ相当イベントからの追従（位置誤差約2.5 mm）、マウス解放後の30 Hz配信、停止・リセットメニューを検証しました。実機でのマーカー操作は未確認です。

## 指先の位置を手動オフセットで指定する

URDFを変更せず、`tip_frame`（既定 `gripper_link`）の原点から指先への距離を、m単位で設定できます。

```bash
ros2 launch so101_follower_moveit servo.launch.py \
  use_fake_hardware:=false interactive_target:=true \
  tip_offset_x:=0.0 tip_offset_y:=0.0 tip_offset_z:=0.08 \
  max_linear_speed:=0.08 max_joint_speed:=0.8 max_joint_acceleration:=1.0 gain:=4.0
```

上の0.08 mは設定例です。SO-101の指先の実測寸法・方向を表す値ではありません。x/y/zは **tip_frameのローカル座標軸** です。手首が回ればオフセットの方向も一緒に回転します。ワールド座標へ単純に固定値を足す処理ではありません。

青い球が実際に追従制御する点です。最初は模擬モードで、RVizのTF軸表示やモデルを見ながら、青い球が指先に重なるように `tip_offset_x/y/z` を調整して再起動してください。緑の操作マーカーの初期位置と右クリックのリセット位置も、このオフセット点になります。外部から配信する目標も、指先として設定したこの点の目標位置です。

位置だけでなく、速度を計算するヤコビアンもオフセット点に合わせています。オフセットはtip_frameに固定された点を表します。グリッパーの開閉で動く片側の指先を厳密に表現する場合、開閉角度に応じた追加モデルが必要です。また、オフセットを指定しても衝突形状自体は追加されません。

## 追従を速くする

既定上限を、手先3→8 cm/s、関節0.3→0.8 rad/s、関節加速度0.5→1.0 rad/s²へ変更しました。近距離での応答を決める `gain` も2→4へ変更しています。いずれもROSの起動引数で調整できます。

これは指令上限です。実際の速度は姿勢、ヤコビアン、`joint_limits.yaml`、Servoの衝突・関節限界チェック、モーター追従能力にも制約されます。起動中の値を変更する方式ではないため、変更時は再起動してください。以前の設定に戻す例：

```bash
ros2 launch so101_follower_moveit servo.launch.py interactive_target:=true \
  max_linear_speed:=0.03 max_joint_speed:=0.3 max_joint_acceleration:=0.5 gain:=2.0
```

オフセット対応の検証では、有限差分ヤコビアン・ローカル座標の回転を含む計8件のユニットテストが成功しました。模擬モードに `tip_offset_x:=0.02 tip_offset_y:=0.01 tip_offset_z:=0.04` を指定し、新しい速度設定でマーカー追従（位置誤差約1.4 mm）と停止・リセットを確認しました。実機での高速動作と指先寸法の適合は未確認です。

`position_singularity` で全停止していた旧処理は修正済みです。位置ヤコビアンのランク不足だけを理由にすべての関節指令を止めず、減衰最小二乗法で求めた速度を上限内で使用します。Servo側の衝突・関節限界チェックは維持しています。障害物の迂回や特異姿勢からの自動的な姿勢変更は行いません。
