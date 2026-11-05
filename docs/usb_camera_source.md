# 以前のUSBカメラ構成を使用する

参照: https://github.com/a-uru/robot_arm/tree/b22e8e2583c0ae753f848bfa0dbed6f8a113cea1/src

参照コミットの3つのソースを `src/` に復元しています。

| リポジトリ | コミット |
| --- | --- |
| ros2_v4l2_camera | 22d6ce190f5caebd20cc35a2635c05b49008f447（0.6.2） |
| image_common | 7a0da75a423c16d4ede762461425966e1fb3e106 |
| vision_opencv | e6b1511e89c119e60f6563e4462c6a598f61872a |

現在のJazzyでは `camera_calibration_parsers` の古いCMakeのYAMLリンク指定が不足するため、`find_package(yaml-cpp REQUIRED)` と `yaml-cpp` へのリンク指定を補正しています。画像処理は追加していません。

## ビルド

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/image_common src/vision_opencv src/ros2_v4l2_camera \
  --build-base build/camera --install-base install/camera \
  --packages-up-to v4l2_camera --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

旧ライブラリを使用するのはカメラ用の別ターミナルに限定します。MoveIt全体の環境へこのoverlayを追加する必要はありません。

## 起動

既存のカメラノードをCtrl+Cで停止してから実行します。

```bash
cd ~/robot_arm
./scripts/start_usb_camera.sh
```

スクリプト内で `install/camera/local_setup.bash` を読み込みます。これまでは `/opt/ros/jazzy` のみを読み込んでいたため、apt版0.7.3を使用していました。

表示側は標準のrqtを使用します。別ターミナルで：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 run rqt_image_view rqt_image_view /image_raw
```

構成はUSBカメラ → 旧版v4l2_camera → `/image_raw` → 標準ビューアです。独自ビューア・画像中継・JPEG変換は追加しません。以前のソース版への復元だけでフリーズが解消するかは、実際の表示で確認が必要です。

DDS転送側の損失については[共有メモリ容量の比較測定](camera_dds_analysis.md)で切り分けました。起動スクリプトがカメラ側の共有メモリを64 MiBへ増やす設定を読み込みます。画像処理は追加していません。

旧版ドライバではCameraInfoのframe_idが未設定だったため、配信時に画像と同じヘッダー（時刻・frame_id）を設定する修正を加えています。画像・CameraInfoとも `camera_optical_frame` を使用します。修正後はカメラノードの再起動が必要です。
