# ロボットアーム開発

## 概要

オープンソースのロボットアームを用いて、ロボットの知能化を研究する個人プロジェクトです。

主な機能は次のとおりです。

- ROS 2 によるロボットアームの制御
- PS4 コントローラーによる遠隔操作（2025年10月30日に実装）
- カメラ映像による動作確認
- RViz によるロボットの状態表示

## 目次

- [初期設定](#初期設定)
- [遠隔操作の手順](#遠隔操作の手順)
- [MoveIt 2 による操作](#moveit-2-による操作)
- [カメラの設定](#カメラの設定)
- [トラブルシューティング](#トラブルシューティング)
- [今回の修正と確認内容](#今回の修正と確認内容)
- [ライセンス](#ライセンス)

## 初期設定

以下では、ワークスペースを `~/robot_arm` とします。別の場所に配置している場合は、パスを読み替えてください。ROS 2 の環境設定を読み込んだターミナルで実行します。

### ディレクトリ構成

```text
~/robot_arm/
├── README.md
├── Lerobot_ros2/
│   └── src/                   # アームの制御・モデル関連パッケージ
├── src/                       # カメラ関連パッケージ
├── build/                     # ビルド結果
├── install/                   # 実行環境とインストール結果
└── log/                       # ビルドログ
```

ビルドと `source install/setup.bash` は、ワークスペースのルートである `~/robot_arm` で実行します。

### Python の依存ライブラリ

モーターブリッジには、`scservo_sdk` を提供する `feetech-servo-sdk` と、辞書比較に使う `deepdiff`、進捗表示に使う `tqdm` が必要です。`ros2 run` が使う `python3` にインストールしてください。

```bash
sudo apt-get install python3-pip
python3 -m pip install --user 'feetech-servo-sdk>=1.0.0,<2' deepdiff tqdm
python3 -c "import scservo_sdk, deepdiff, tqdm; print(scservo_sdk.__file__)"
```

Ubuntu で `externally-managed-environment` が表示される場合は、ユーザー領域へのインストールに次のコマンドを使用します。

```bash
python3 -m pip install --user --break-system-packages 'feetech-servo-sdk>=1.0.0,<2' deepdiff tqdm
```

### シリアルポートの権限

USB 接続したアームを操作するため、ユーザーを `dialout` グループに追加します。

```bash
sudo usermod -aG dialout "$USER"
```

Ubuntu からログアウトして再ログインし、所属グループとデバイスを確認してください。

```bash
id -nG
ls -l /dev/ttyACM*
```

## 遠隔操作の手順

各ノードは別々のターミナルで起動してください。各ターミナルで ROS 2 の環境設定とワークスペースの `install/setup.bash` を読み込みます。

### 1. PS4 コントローラーの接続

PS4 コントローラーを接続し、入力を受け取るノードを起動します。

```bash
ros2 run joy joy_node
```

### 2. ロボットアームへの接続

**接続するとアームが初期位置へ動きます。周囲を確認してから実行してください。**

```bash
cd ~/robot_arm
source install/setup.bash
ros2 run so101_hw_interface so101_motor_bridge
```

既定のシリアルポートは `/dev/ttyACM0` です。異なるポートを使う場合は、次のように指定します。

```bash
ros2 run so101_hw_interface so101_motor_bridge --ros-args -p port:=/dev/ttyACM1
```

### 3. RViz の起動

ロボットアームの状態を表示します。

```bash
cd ~/robot_arm
source install/setup.bash
ros2 launch so101_follower_description display.launch.py \
    use_gui:=false \
    joint_states_topic:=/so101_follower/joint_states
```

### 4. 操作プログラムの起動

操作用のプログラムを起動します。

```bash
cd ~/robot_arm/Lerobot_ros2/src/so101_hw_interface/controll/
python3 keyboard_controller.py
```

## MoveIt 2 による操作

ROS 2 Jazzy で、アームの5関節とグリッパーの1関節を制御します。軌道は MoveIt 2 → `JointTrajectoryController` → トピック接続 → `so101_motor_bridge` の順に実機へ渡します。

### 依存パッケージとビルド

```bash
sudo apt-get install -y ros-jazzy-moveit ros-jazzy-joint-state-topic-hardware-interface ros-jazzy-ros2-controllers
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --build-base build/moveit --packages-select so101_follower_description so101_hw_interface so101_follower_moveit --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`build/moveit` は、階層変更前のパスを記録したビルドキャッシュと分けるために使用しています。Python の依存ライブラリと `dialout` の設定も、[初期設定](#初期設定)に従って済ませてください。

### 模擬アームで計画・実行を試す

```bash
ros2 launch so101_follower_moveit demo.launch.py
```

既定では模擬ハードウェアを使い、実機には接続しません。RViz の「MotionPlanning」で「Planning Group」を `arm` にし、目標の関節角度を設定して「Plan」、続けて「Execute」を押します。グリッパーは `gripper` グループで操作します。

アームは5軸のため、逆運動学は位置のみを対象としています。手先の位置と任意の3軸姿勢を同時に指定することはできません。最初は関節角度による目標指定で動作を確認してください。

### 実機を動かす

先に起動した `so101_motor_bridge` やコントローラー、遠隔操作プログラムを停止し、アームの周囲を確認してください。実機モードではモーターブリッジも一緒に起動します。

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_follower_moveit demo.launch.py use_fake_hardware:=false port:=/dev/ttyACM0
```

この起動方法では、接続時に可動範囲の中央へ移動する処理を無効にし、最初に読み取った現在位置を保持します。6関節の位置情報を受信してから軌道コントローラーを起動します。RViz で計画を確認し、「Execute」で実機へ指令を送ってください。

付属の `so101_calibration.yaml` に合わせて関節可動範囲を設定し、計画上限を速度 0.5 rad/s、加速度 0.5 rad/s²、既定の倍率をそれぞれ 0.1 にしています。キャリブレーションは実機固有のため、別のアームや再調整後は、キャリブレーションと URDF・`joint_limits.yaml` の可動範囲を一致させてください。

独自のキャリブレーションを使う場合は `calib_file:=/絶対パス/設定.yaml` を追加できます。既に別途ブリッジを起動している場合は `start_motor_bridge:=false` を追加してください。その場合も、ブリッジは `--ros-args -p move_to_midpoint:=false` を付けて起動してください。画面なしで起動する場合は `use_rviz:=false` を追加します。

### 起動状態の確認

別のターミナルで環境設定を読み込んで確認します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash
ros2 control list_controllers
ros2 action list
ros2 topic echo /joint_states --once
```

`joint_state_broadcaster`、`arm_controller`、`gripper_controller` がすべて `active` になり、次のアクションが表示されれば軌道を受け付ける状態です。

- `/arm_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

位置情報待ちで終了する場合は、シリアルポート、アクセス権限、モーターブリッジのエラーを確認してください。

模擬モードでは、アーム・グリッパー両方で MoveIt による計画、軌道実行、関節位置の到達を確認しました。実機用のトピック接続についても、模擬フィードバックで現在位置の保持と計画・実行を確認しました。シリアル接続した実機での軌道実行は未確認です。

## カメラの設定

### 接続されているカメラの確認

```bash
v4l2-ctl --list-devices
```

これまでの環境で確認したデバイスは次のとおりです。番号は接続状態によって変わるため、実際の出力を確認してください。

- 内蔵カメラ：`/dev/video0`、`/dev/video1`
- 外付けカメラ XWF-1080p6：`/dev/video2`、`/dev/video3`

### カメラノードの起動

外付けカメラ XWF-1080p6 の場合：

```bash
cd ~/robot_arm
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2
```

内蔵カメラの場合：

```bash
cd ~/robot_arm
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### 映像の表示

#### rqt_image_view を使う場合

```bash
cd ~/robot_arm
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

左上の選択欄で `/image_raw` を選びます。

#### RViz を使う場合

1. 左下の「Add」をクリックします。
2. 「By display type」から「Image」を選びます。
3. 「Image Topic」に `/image_raw` を指定します。

### 配信されるトピック

| トピック | 内容 | メッセージ型 |
| --- | --- | --- |
| `/image_raw` | カメラ画像（従来の設定では解像度 640×480、形式 rgb8） | `sensor_msgs/msg/Image` |
| `/camera_info` | カメラのキャリブレーション情報 | `sensor_msgs/msg/CameraInfo` |

## トラブルシューティング

### `No module named 'scservo_sdk'`

モーター通信用 SDK が、実行中の Python 環境に入っていない場合に発生します。[Python の依存ライブラリ](#python-の依存ライブラリ)の手順で `feetech-servo-sdk` をインストールしてください。

インストール後は、モーターブリッジを再実行します。

```bash
ros2 run so101_hw_interface so101_motor_bridge
```

### `/dev/ttyACM0` の `Permission denied`

シリアルポートへのアクセス権限が不足しています。[シリアルポートの権限](#シリアルポートの権限)の手順で `dialout` グループに追加し、ログアウト・再ログインしてください。

デバイス自体が存在しない場合は USB 接続を確認し、`ls -l /dev/ttyACM*` で実際のポート名を調べてください。

### `sync_read failed: dict expected at most 1 argument, got 2`

`deepdiff` が未導入のとき、従来のコードが代わりに使っていた `dict` 派生クラスへ辞書を2つ渡してしまうことが原因でした。

現在はこの代替処理を削除し、正式な `deepdiff` を使うように修正しています。[Python の依存ライブラリ](#python-の依存ライブラリ)の手順で依存ライブラリを導入し、実行中のノードを `Ctrl+C` で停止してから再起動してください。

今回の環境ではビルド先がソースへのシンボリックリンクになっているため、再ビルドせず修正が反映されます。コピー形式でビルドした環境では、ワークスペースで再ビルドして環境設定を読み込み直してください。

```bash
cd ~/robot_arm
colcon build --packages-select so101_hw_interface
source install/setup.bash
```

### カメラ映像が表示されない

トピックの配信状況を確認します。

```bash
# 画像関連のトピックを確認
ros2 topic list | grep image

# 画像の配信頻度を確認
ros2 topic hz /image_raw

# メッセージを1件確認
ros2 topic echo /image_raw --once
```

- デバイスが見つからない場合は、カメラの接続とデバイス番号を確認してください。
- アクセス権限がない場合は、`sudo usermod -aG video "$USER"` で `video` グループに追加し、ログアウト・再ログインしてください。
- フレームレートが低い場合は、解像度を下げるか USB の帯域を確認してください。

## 今回の修正と確認内容

2026年10月28日に、モーターブリッジの起動と読み取りに関する問題へ次の対応を行いました。

- `feetech-servo-sdk` 1.0.0 を Python 3.12 のユーザー領域に導入し、`scservo_sdk` の読み込みとバスの初期化が成功することを確認しました。
- `/dev/ttyACM0` のアクセス権限エラーに対し、`dialout` グループへの追加と再ログインの手順を案内しました。グループ変更の実行はユーザー側で行う手順です。
- `deepdiff` と `tqdm` をユーザー領域に導入し、`motor_bridge.py` にあった両ライブラリの代替処理を削除しました。
- `setup.py` の依存関係に `feetech-servo-sdk`、`deepdiff`、`tqdm` を追加しました。
- 制御テーブルが同じ場合と異なる場合の比較処理が、どちらも正常に判定できることを確認しました。

確認時はシリアルポートを開いていません。実機への接続、連続読み取り、アームの動作は別途確認が必要です。

## ライセンス

本プロジェクトはオープンソースです。詳細は各パッケージのライセンスを確認してください。

### 開始・停止時の動きを滑らかにする設定

OMPL の時間パラメーター計算後に Ruckig による軌道平滑化を適用しています。各関節の加速度の変化率（ジャーク）は `joint_limits.yaml` の `max_jerk: 1.0` rad/s³ を初期調整値としています。実機性能を測定した上限ではありません。設定変更後は MoveIt を再起動し、軌道を再計画してください。実機の引っ掛かりが残る場合は、低い速度・加速度倍率で再確認し、モーター側の加減速設定や機械的な遊びも切り分けてください。

開始・停止時の指令段差と倍率1.0時の速度については、[実機動作の調査記録](docs/so101_motion_investigation.md)に、ログとの照合結果、実行時間監視の修正、速度比較をまとめています。
