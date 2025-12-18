# ロボットアーム開発

## 概要
オープンソースのロボットアームを用いて、個人でロボットの知能の研究をするプロジェクト

2025/10/30ではPS4コントローラーからのテレオペレーションが出来た

2026/6までは学ロボで忙しいため進捗は遅い

## 使い方(テレオペ)
1. PS4コントローラーの接続:
```bash
ros2 run joy joy_node
```
2. 実機との接続(ロボットアームが初期位置に移動するので注意)
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run so101_hw_interface so101_motor_bridge
```
3. rvizによる可視化
```bash
cd ~/robot_arm
source install/setup.bash
ros2 launch so101_follower_description display.launch.py     use_gui:=false     joint_states_topic:=/so101_follower/joint_states
```
4. コントロールノードの起動
```bash
cd ~/robot_arm/Lerobot_ros2/src/so101_hw_interface/controll/
python3 keyboard_controller.py
```

## 使い方(カメラ)

### 利用可能なカメラデバイスの確認
システムに接続されているカメラを確認:
```bash
v4l2-ctl --list-devices
```

現在のシステムでは以下のカメラが利用可能:
- **Integrated Camera** (内蔵カメラ): `/dev/video0`, `/dev/video1`
- **XWF-1080p6 Camera** (外付けカメラ): `/dev/video2`, `/dev/video3`

### カメラノードの起動
外付けカメラ (XWF-1080p6) を使用する場合:
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2
```

内蔵カメラを使用する場合:
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### カメラ映像の確認
カメラノード起動後、以下のいずれかの方法で映像を確認できます:

#### 方法1: rqt_image_view を使用
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
起動後、ウィンドウ左上のドロップダウンメニューから `/image_raw` を選択

#### 方法2: RViz を使用
RVizを起動後:
1. 左下の「Add」ボタンをクリック
2. 「By display type」→「Image」を選択
3. 追加したImageディスプレイの「Image Topic」を `/image_raw` に設定

### カメラのトピック
- `/image_raw`: カメラ画像 (sensor_msgs/msg/Image)
  - 解像度: 640x480
  - エンコーディング: rgb8
- `/camera_info`: カメラ情報 (sensor_msgs/msg/CameraInfo)

### トラブルシューティング
映像が表示されない場合:
```bash
# トピックが配信されているか確認
ros2 topic list | grep image

# 画像データが流れているか確認
ros2 topic hz /image_raw

# 画像メッセージの内容を確認
ros2 topic echo /image_raw --once
```
