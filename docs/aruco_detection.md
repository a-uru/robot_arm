# ArUcoマーカー検出と3D位置・姿勢

`so101_aruco` は `DICT_4X4_50` のマーカーを検出し、校正済みカメラに対する位置（m）・姿勢（Quaternion）を配信します。ロボットは動かしません。`base_link` への変換は今後の段階です。

USBカメラではFast DDSを使用し、Zenohルーターは不要です。各ターミナルでROSの環境を読み込んだ後に `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` を設定し、`ROS_DOMAIN_ID` も揃えてください。通信方式の切り替え後は `ros2 daemon stop` を実行します。

現在のXWF-1080p6・640×480構成は内部校正済みです。通常は[READMEの起動手順](../README.md#usbカメラの起動)で `config/camera/ost.yaml` を読み込みます。以下の内部校正は、条件を変えた場合の再実施手順です。ArUcoの実画像によるPoseと精度の確認は未完了です。

## 1. 依存パッケージとビルド

```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-scipy \
  ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-v4l2-camera \
  ros-jazzy-camera-calibration ros-jazzy-rqt-image-view
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_aruco --symlink-install --build-base build/aruco
source install/setup.bash
```

UbuntuのOpenCVを使います。ROS用Pythonへpip版OpenCVを重ねて入れる必要はありません。

## 2. マーカーの印刷

通常は [A4印刷用PDF（ID 0・1、黒枠20 mm）](aruco_markers/DICT_4X4_50_20mm_A4.pdf) を使います。

1. PDFをダウンロードしてPDFビューアで開きます。
2. 用紙をA4、倍率を「実際のサイズ」または「100%」に設定します。「用紙に合わせる」「余白に合わせる」は無効にします。
3. 白い紙に印刷し、黒枠の外側の一辺が20 mm、確認用スケールが50 mmであることを定規で確認します。
4. 黒枠の周囲に5 mm程度の白い余白を残して切り、硬く平らな板に貼ります。光沢テープを印刷面に重ねないでください。

プログラムの既定値は `marker_size=0.02`（m）、ID 0・1も各0.02 mです。白い余白を含む紙全体の寸法ではありません。実寸がずれた場合は印刷倍率を修正して再印刷します。


[ID 0・黒枠20 mm](aruco_markers/DICT_4X4_50_id0_20mm.svg) と [ID 1・黒枠20 mm](aruco_markers/DICT_4X4_50_id1_20mm.svg) を用意しています。SVGをブラウザなどで開いて倍率100%で印刷し、**黒枠の外側一辺が20 mmであることを実測**してください。用紙に合わせた拡大縮小は無効にし、白い余白を残して平らで反射しにくい板へ貼ります。PNGは画像表示・検出確認用で、印刷寸法は保証しません。

マーカーサイズの指定は白い余白を除く黒枠の辺長です。寸法の設定誤差は距離の誤差につながります。

## 3. 校正前でも検出を確認

カメラのデバイス番号を `v4l2-ctl --list-devices` や `ls /dev/video*` で確認します。以下のターミナルではそれぞれROSとワークスペースをsourceしてください。

```bash
ros2 launch so101_aruco detect.launch.py video_device:=/dev/video2
```

別ターミナル：

```bash
ros2 run rqt_image_view rqt_image_view
```

`/aruco/image` を選ぶと検出枠とIDが表示されます。校正前は警告を表示し、3D位置・姿勢とTFは配信しません。`/camera_info` が存在するだけでは校正済みとは限りません。

## 4. カメラ内部キャリブレーション

校正にはArUcoの2枚とは別にチェッカーボードを用意します。例は**内側の交点10×7、正方形の辺24 mm**です（マス数は11×8）。使用するボードに合わせて値を変えてください。

カメラを起動したまま、別ターミナルで：

```bash
ros2 run camera_calibration cameracalibrator --size 10x7 --square 0.024 --no-service-check --ros-args -r image:=/image_raw
```

`--no-service-check` は校正結果をドライバへ直接書き込むサービスの起動時確認を省略します。画像からの校正と `SAVE` によるファイル保存は利用できます。ここでは `COMMIT` は使用せず、保存したYAMLをカメラ起動時に読み込みます。

ボードを画面の隅・中央、異なる距離・角度で撮影し、GUIの進捗が十分になったら `CALIBRATE` → `SAVE`。通常 `/tmp/calibrationdata.tar.gz` が保存されます。GUIが示す保存先を確認し、YAMLをワークスペース内へ取り出します。

```bash
mkdir -p ~/robot_arm/config/camera
tar -xzf /tmp/calibrationdata.tar.gz -C ~/robot_arm/config/camera ost.yaml
```

カメラを停止して、保存した校正を読み込んで再起動します。

```bash
cd ~/robot_arm
./scripts/start_usb_camera.sh -p camera_info_url:=file:///home/akira/robot_arm/config/camera/ost.yaml
```

ドライバに校正ファイルのcamera_name不一致の警告が出た場合は、ドライバのcamera_nameとYAMLのcamera_nameを合わせてください。校正時と運用時の解像度・フォーカスを合わせ、可能ならオートフォーカスを固定します。`/image_raw` と `/camera_info` は同じ光学座標frame_idである必要があります。起動ファイルでは `camera_optical_frame` を指定しています。

## 5. 位置・姿勢を確認

```bash
ros2 topic echo /aruco/detected_ids
ros2 topic echo /aruco/marker_0/pose
ros2 topic echo /aruco/marker_1/pose
ros2 run tf2_ros tf2_echo camera_optical_frame aruco_marker_0
```

- `/aruco/detected_ids`: その画像で検出したID。未検出時は空配列。
- `/aruco/marker_<ID>/pose`: 校正条件・再投影誤差の条件を満たすマーカーの `PoseStamped`。
- `/aruco/image`: 検出枠、座標軸、カメラZ方向距離、再投影誤差（px）。
- `/tf`: カメラ光学座標 → `aruco_marker_<ID>`。画像の時刻を使用。

カメラ光学座標は **Xが画像右、Yが画像下、Zがカメラ前方**。距離の単位はmです。マーカー原点は中心、Xは印刷面の右、Yは上、Zは印刷面の手前です。姿勢はこのマーカー座標からカメラ座標への回転です。

未検出・姿勢が棄却された場合、そのマーカーのPose/TFは新規配信されません。購読側は必ずタイムスタンプを確認し、古い値を現在位置として使わないでください。正面に近い平面マーカーの姿勢には曖昧さがあり、再投影誤差が小さくても姿勢が揺れる場合があります。

RVizではFixed Frameを `camera_optical_frame` にしてTFを追加します。ロボット座標へ接続する前は `base_link` との関係はありません。

## 既存カメラ配信を使う

カメラを重複起動しないように：

```bash
ros2 launch so101_aruco detect.launch.py start_camera:=false \
  image_topic:=/image_raw camera_info_topic:=/camera_info marker_size:=0.02
```

異なる寸法のIDを使う例（各パラメータは起動時に指定）：

```bash
ros2 run so101_aruco detector --ros-args \
  -p marker_ids:='[0, 1]' -p marker_sizes:='[0.02, 0.03]' -p marker_size:=0.02
```

リストにないIDには `marker_size` が適用されます。画像は未補正のraw画像を使用します。魚眼モデル・ROI・ビニングは現在対応していません。

## 検証

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH="src/so101_aruco:$PYTHONPATH" python3 -m pytest src/so101_aruco/test -q
```

合成マーカー画像のID検出と、既知のカメラ・位置・姿勢から投影した四隅を用いたメートル単位の姿勢復元を検証します。実カメラの精度は校正・印刷寸法・照明・撮影距離に依存するため、既知の距離で実測してください。

実装環境ではビルド・上記2テスト・ROSコールバックによる未校正時の抑止と校正時のPose/TF送信処理・launchからのノード起動を確認済みです。ただし、環境のネットワーク制限によりDDSのプロセス間通信は未検証で、カメラデバイスも未接続です。カメラドライバ・校正ツール・ビューアのapt導入はsudoパスワードが必要なため、この環境では未完了です。

USBカメラは標準ドライバから `/image_raw` を直接配信します。独自ビューア・独自DDSプロファイルは不要です。
