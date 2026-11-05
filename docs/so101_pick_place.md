# ArUcoの物体位置をEnterで確定して把持・搬送

カメラ・ArUco・MoveIt・コントローラー・RViz・Enter受付を一つのlaunchで起動する。起動しただけでは動作しない。Enter時に、同一画像のID 0・ID 1から得た新しく安定した `/aruco/object/pose` を確定し、全工程の計画が成功した後に実行する。

## 1. 調整ファイル

[pick_place.yaml](../src/so101_pick_place/config/pick_place.yaml) を編集する。搬送先・開閉角・指先位置は未確定のため、現在は仮値と `configuration_ready: false` にしている。

| パラメータ | 内容 |
| --- | --- |
| `place_xyz_m` | base_link基準の搬送先。指先の把持中心（TCP）が解放する位置、m |
| `gripper_open_rad` / `gripper_closed_rad` | URDFのgripper関節角、rad。現モデルの範囲は−0.222427〜0.868233 |
| `tip_link` | TCPオフセットの基準リンク。既定gripper_link |
| `tcp_offset_m` | tip_link原点から実際の指の把持中心まで。リンクローカル座標、m |
| `grasp_offset_object_m` | 物体原点から把持点まで。物体ローカル座標、m |
| `approach_height_m` | 把持位置の上方へ接近する高さ。base_linkの＋Z方向 |
| `lift_height_m` | 把持後の持上げ高さ。把持位置から＋Z |
| `place_approach_height_m` | 搬送先へ近づく際の上方余裕 |
| `retreat_height_m` | 解放後の上方退避量 |
| `velocity_scaling` / `acceleration_scaling` | MoveItの倍率。既定各0.1 |
| `workspace_min_m` / `workspace_max_m` | TCP目標を許可する直方体。全接近・持上げ・搬送・退避点を検査 |
| `table_enabled` / `table_top_z_m` 等 | MoveItへ登録する机の寸法・位置。実測後に設定して有効化 |
| `payload_radius_m` | 搬送物の衝突形状。TCP中心の球による近似、既定18 mm半径＋把持オフセット長 |
| `configuration_ready` | 上記を設定したらtrue。falseの間はEnterを拒否 |
| `tcp_offset_confirmed` | 指先位置を実測したらtrue。実機モードでは必須 |

ID 0取付オフセットとID 1物体原点の設定は引き続き [marker_offsets.yaml](../src/so101_aruco/config/marker_offsets.yaml)。`base_offset_calibrated: true` によりbase_link基準の物体Poseを配信する。

`place_xyz_m` はTCPの位置。`grasp_offset_object_m`がゼロなら物体原点をその位置へ持っていく想定だが、把持ずれ・滑り・手先姿勢による差は別途確認が必要。物体オフセットが非ゼロの場合は物体原点と解放時のTCPは一致しない。

パラメータは起動時に読み込む。編集後はlaunchを再起動する。実機用TCPは推測で設定していない。模擬テストで使った `[0, 0, -0.1]` mはテスト条件であり、実機の校正値ではない。

## 2. ビルド

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select so101_pick_place --symlink-install --build-base build/pick_place
source install/setup.bash
```

既存の `so101_servo` のURDF運動学を指先IK計算に再利用するが、Servoノードを起動したりServo指令を送るわけではない。必要に応じて既存パッケージもビルドする。

## 3. 一括起動

先に個別起動したカメラ・ArUcoプレビュー・MoveIt・モーターブリッジ・Servo・仮joint_state_publisherを停止する。重複したTFや複数の制御元を作らないため。

模擬アームで起動（カメラは実カメラを使う）：

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch so101_pick_place pick_place.launch.py
```

実機で起動：

```bash
ros2 launch so101_pick_place pick_place.launch.py use_fake_hardware:=false port:=/dev/ttyACM0
```

これだけで以下を起動する。

- 校正済みUSBカメラ（既存start_usb_camera.sh、ost.yaml、SHM 64 MiB）
- 既存のArUco推定と原点オフセット
- MoveIt MoveGroup、robot_state_publisher、ros2_controlとarm/gripperコントローラー
- RViz（ロボット・TF・物体立方体）
- Enter受付とピッキングノード

RVizのFixed Frameはworld。現在のURDFではworldとbase_linkはゼロ固定変換。目標は常にbase_link基準。通常のプレビューと異なり、実機モードのモデル関節値は実機フィードバックから取得する。

把持点は `tip_link`（既定 `gripper_link`）→ `tcp_link` の固定TFで出力する。位置は `tcp_offset_m`、軸の向きは `tip_link` と同じ。RVizのTF表示で `tcp_link` を有効にして確認できる。オフセット変更後はlaunchを再起動する。既定の `[0, 0, 0]` では両リンクの原点が重なる。

別ファイルを使う場合：

```bash
ros2 launch so101_pick_place pick_place.launch.py pick_config:=$HOME/robot_arm/src/so101_pick_place/config/pick_place.yaml offsets_file:=$HOME/robot_arm/src/so101_aruco/config/marker_offsets.yaml
```

`workspace` は既存カメラスクリプト・校正ファイルがあるルート（既定はこのワークスペース）。`start_camera:=false` は既存カメラを使う場合、`start_perception:=false` は既存の物体Pose配信を使う場合。`use_rviz:=false` でGUIを省略できる。実機への接続を避けたいときは `use_fake_hardware:=true` を使用する。

## 4. Enterで開始

1. `configuration_ready` をtrueにして上記launchを起動。実機では `tcp_offset_confirmed` もtrueにする。
2. 両マーカーを映して、直近0.5秒程度の物体位置が安定するのを待つ。
3. **launchを実行しているターミナルを選択し、Enterを1回押す**。RVizウィンドウ内のEnterではない。
4. 最新位置を固定し、全9工程を事前計画する。計画中・実行中の追加Enterは無視する。
5. 全計画が成功したら、各工程の直前に最新の実測関節角から再計画して実行。事前計画の軌道は実行に流用しない。再計画に失敗したら後続工程を停止する。最後に退避してDONEとなり、次のEnterを受け付ける。

工程：開く → 上方へ接近 → 把持位置 → 閉じる → 持上げ → 搬送先上方 → 配置位置 → 開く → 退避。

Enterはlaunch配下の標準入力ではなく制御端末 `/dev/tty` から読む。端末のない環境や自動検証用には同じ処理のサービスを用意している：

```bash
ros2 service call /pick_place/trigger std_srvs/srv/Trigger '{}'
ros2 topic echo /pick_place/status
```

**triggerサービスもEnterと同様に、実機モードでは動作を開始する。** 起動確認用の読み取り専用サービスではない。

## 5. 目標の有効性と中断

Enter時は新しい同一画像由来のPose・更新するタイムスタンプ・観測窓のサンプル数・位置変動・作業範囲を検査する。MoveItへは現在の全関節値から全工程を計画し、搬送先で解がない場合も最初の動作を始めない。実行時は各工程を最新の全関節実測値から再計画し、その始点と実フィードバックを照合する。計画中にアームが動けば開始判定で停止する場合がある。把持への移動は従来の許容値を維持する。各工程後にTCPまたはグリッパー角の到達を確認する。

把持点はEnter時に固定し、動作中の映像へ追従しない。接近・閉動作前に新しい観測があり物体が大きく移動していれば中断する。手で隠れて検出できない場合は、Enter時の静止物体位置を使い続ける。

中断要求：

```bash
ros2 service call /pick_place/cancel std_srvs/srv/Trigger '{}'
```

全体停止はlaunchターミナルのCtrl+C。中断や動作失敗後は後続工程を進めず、グリッパーを自動で開かない。動作開始後の異常・中断は再開をラッチし、状態を確認してlaunchを再起動する。閉動作後の異常では、物体を保持している可能性があるため物体とアームの状態を確認して復旧する。キャンセルはROS/コントローラーへの要求であり、機械的な即時停止やハードウェア非常停止を代替するものではない。

## 6. 適用範囲

- 現在は5軸の**位置のみ**の目標。任意の物体姿勢へグリッパー姿勢を合わせる機能は含まない。最初は置き方・把持方向を揃え、指先位置と実際の指の向きを確認する。
- 指先オフセット込みの位置IKは既存URDFの関節候補を数値計算し、MoveItの状態妥当性サービスで衝突を確認。その候補間の軌道をMoveItで計画・実行する。実行後も指先位置を実TFから検証する。
- 経路はOMPLの関節軌道であり、接近や持上げが厳密な直線になる保証はない。近接作業に必要な直線接近や姿勢拘束は別途対応が必要。
- 机は実測後に有効化する。登録していない周囲物体は衝突判定に含まれない。作業範囲の検査は目標点に対するもので、経路全体をその直方体に拘束するものではない。
- 持上げ以降は20 mm物体を包含する球を手先へ付けて衝突判定する。実物の形状・把持点が変われば球半径も調整する。
- グリッパー角に到達したことは物体を把持できた証明ではない。把持力・滑り・落下検出は未実装。手先・マーカーの校正誤差も残る。

## 検証記録

2026年11月6日、実機と異なるROS Domain 73・mock_components/GenericSystemで検証。カメラ・シリアル機器は起動せず、同時検出由来Poseに相当する合成Poseを配信した。

- コア・失敗時の順序・TCP IKの9件のテスト成功。
- 実際のMoveIt/ros2_control模擬環境で全9工程の事前計画と実行がDONEまで完了。
- 搬送物の付加・解放時の除去、各工程の関節/TCP到達確認を含む。
- 古い目標と重複トリガーの拒否を確認。模擬移動中に取消し、閉じる・解放へ進まず中断したことと、中断後の実行異常ラッチによる再開始拒否を確認。

記録は `log/pick_place_sim_success.json` と `log/pick_place_cancel_result.json`（Git管理外）。実機での把持・搬送は未実施。

模擬検証では終了時に既存MoveGroupプロセスのデストラクタでsegmentation faultが出る場合もあった。ros2_controlとrobot_state_publisherは停止し、ピッキングノードも終了している。これは成功した計画・実行の途中の異常ではないが、終了処理の問題として残る。

### 関節の追従誤差を記録する

実機用launchと同じROS_DOMAIN_IDの別ターミナルで、動作開始前に次を実行する。購読のみで、動作指令は送らない。

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/record_arm_tracking.py --seconds 180
```

記録中にlaunch側でEnterを押し、中断後も10秒ほど待つ。`log/arm_tracking/<日時>/` に `controller.csv`（目標・実測・誤差・速度）、`bridge.csv`（モーターへの指令・実測）、`events.csv`（工程・停止理由）、終了時に `summary.json` を保存する。Ctrl+Cで早期終了しても保存する。実機／模擬の区別は使用したlaunch設定で確認する。

停止直前の目標が一定の区間で、実測が目標へ近づき続けていれば到達の遅れ、実測も一定で誤差が残れば定常的なずれを疑う。中断時にはコントローラーが目標を変更することがあるため、中断後の誤差だけで判断しない。モーター指令の校正範囲による制限も確認する。記録だけで負荷・トルク不足を断定することはできない。


### 把持以外の工程の到達許容値

`grasp` は `grasp_joint_tolerance_rad`（既定0.05rad、従来と同じ）と `reached_tolerance_m`（既定10mm）で判定する。グリッパー開閉の判定も維持する。

`approach`、`lift`、`transfer`、`place`、`retreat` は `transit_joint_tolerance_rad: 0.10`（約5.7°）と `transit_reached_tolerance_m: 0.030`（3cm）を使用する。配置位置も3cm以内のずれを許容する。計画時のIK精度は変更せず、全工程の事前計画と工程直前の実測からの再計画も維持する。

すべてのアーム移動はMoveItが計画した軌道を `/arm_controller/follow_joint_trajectory` へ直接送り、工程ごとの関節到達許容値を指定する。経路中の関節誤差0.2rad、終端の追加待機2秒は維持する。MoveItのExecuteTrajectoryによる実行中監視はこの経路では経由せず、コントローラーの結果・時間制限・キャンセルと、実行後の新鮮な関節状態／TFによるTCP到達判定を使用する。コントローラーが失敗した場合は許容せず停止する。計画中の衝突確認は引き続きMoveItで行う。

設定変更後はlaunchを再起動する。許容値を超える誤差・通信異常・計画失敗は従来通り停止する。実測から求めるTCPはエンコーダーとURDFに基づき、物理的な把持成功や物体保持を直接測るものではない。

把持物の球モデル `so101_pick_payload` は搬送中の衝突判定用。解放後は付属物の解除を適用し、その結果worldへ移された同IDの球も続けて削除する。解除だけでは球がグリッパー付近の障害物として残り、退避開始時に自己接触で停止するため。実際に置かれた物体を永続的な障害物として管理する機能は含まない。他の障害物やロボットの衝突判定は変更しない。

把持への移動もコントローラーへ直接実行し、MoveIt ExecuteTrajectory側の固定0.01rad開始判定との不整合を避ける。実行直前は `start_joint_tolerance_rad` で計画始点と新鮮な実測を照合し、超過時は停止する。これは把持の終端精度を緩和する変更ではない。
