# ID 0とID 1の原点オフセット

調整ファイルは [marker_offsets.yaml](../src/so101_aruco/config/marker_offsets.yaml)。ID 0は台座に固定した基準マーカー、ID 1は物体マーカー。いずれも黒枠20 mm。カメラと内部校正の構成はそのまま使用する。

## パラメータの定義

| パラメータ | 意味 |
| --- | --- |
| `base_to_reference.xyz_m` | base_linkから見たID 0の黒枠中心の位置。base_linkの軸で表す、m |
| `base_to_reference.rpy_deg` | ID 0座標のベクトルをbase_link座標へ回す回転。roll/pitch/yaw、度 |
| `marker_to_object.xyz_m` | ID 1から見た、使いたい物体原点の位置。ID 1の軸で表す、m |
| `marker_to_object.rpy_deg` | 物体原点の軸をID 1座標へ回す回転。roll/pitch/yaw、度 |
| `base_offset_calibrated` | 実測・表示確認後にtrue。falseはプレビューのみ |
| `publish_camera_tf` | 校正確認後、base_link → カメラ光学座標のTFを配信するか |

全てのゼロは未測定の初期値。マーカーを貼っただけでbase_linkとの位置関係が分かるわけではない。設定は起動時に読み込むので、編集後はArUcoノードだけ再起動する。

姿勢は `R = Rz(yaw) Ry(pitch) Rx(roll)`。3次元ベクトルの単純加算ではなく、回転込みの剛体変換で計算する。マーカーの原点は黒枠の中心、Xは正立した印刷面の右、Yは上、Zは印刷面から手前。実際の向きは検出画像の軸（赤X・緑Y・青Z）で確認する。紙を回して貼ると軸も回る。

## base_linkとID 0の位置を慎重に決める

1. ID 0が**可動部ではなく台座に剛固定**されていることを確認する。台座と一緒に動くことが必要。肩などの可動リンクに付いている場合、この固定オフセット方式では扱えない。
2. RVizでロボットモデルを表示し、TF表示の `base_link` 原点と軸を確認する。台座の見た目の中心・机の高さを自動的に原点だと思わない。URDFの `visual/origin` や `inertial/origin` はメッシュ・重心のオフセットであり、base_link原点ではない。
3. モデル/CADの既知の寸法と実機の取付穴・面などを対応させ、base_link原点を特定する。本リポジトリのURDFではworld→base_linkはゼロ変換だが、それだけでは台座上の物理的位置を特定できない。取付位置の写真や寸法なしに数値は決められない。
4. base_linkの各軸に沿って、ID 0の**黒枠中心**までのx/y/zを測る。紙や取付板の厚みも含め、マーカーがある平面までを測る。数値をmmで測ったら1000で割ってmにする。
5. マーカーの3軸がbase_linkのどちらを向くかを確認し、rpyを設定する。貼付面だけでなく面内の回転も必要。ゼロ回転は「マーカーのX/Y/Zがbase_linkのX/Y/Zと同じ向き」の場合に限る。
6. `base_offset_calibrated: false` のまま起動し、RVizのFixed Frameを `camera_optical_frame` にしてTFを表示する。`base_link_predict` が、検出したID 0から逆算したbase_link原点。XYZと姿勢を調整し、物理的に測定した値と整合するか確認する。この段階ではロボットのbase_linkへは接続しない。
7. 実測と向きを確認したら `base_offset_calibrated: true` にし、ArUcoを再起動。RVizのFixed Frameを `base_link` にして、ロボットとマーカーの配置が一致するか確認する。既知の点へ物体を置き、作業範囲の複数箇所で測定誤差を確認する。実機を動かす前にこの確認を行う。

もし「ID 0から見たbase_link」の値を測った場合、それをそのまま `base_to_reference` に入れない。測定した変換を `[R, t]` とすると、YAMLへ入れる逆変換は `[R^T, -R^T t]`。回転がある場合、xyzの符号だけを反転しても正しくない。

例（説明用、実機値ではない）：base_linkから見たマーカー中心が `[0.030, -0.040, 0.050]` mなら、そのままxyzへ設定する。向きは実際の取付方向を調べて別途設定する。

## 物体原点の調整

ID 1の黒枠中心から、物体の中心・把持したい点などへ向かう位置を、**ID 1の軸方向で**測る。例えば物体上面へマーカーを貼り、原点が面の内側10 mmにあるなら、マーカーZが外向きであることを確認したうえで `marker_to_object.xyz_m: [0.0, 0.0, -0.010]` とする。これはグリッパーのTCPや把持姿勢の校正とは別。

## 起動と表示

校正情報付きのカメラはREADMEの手順で起動したまま、別ターミナルで：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 launch so101_aruco detect.launch.py start_camera:=false offsets_file:=$HOME/robot_arm/src/so101_aruco/config/marker_offsets.yaml
```

`offsets_file` を省略するとインストール先の同名YAMLを読み込む。編集先を明確にするため、上記のソースファイル指定を推奨する。

- `base_link_predict`: ID 0から推定したbase_link原点のプレビュー。未校正でも表示。
- `aruco_object_origin`: ID 1のオフセット付き物体原点。
- `/aruco/object/pose`: **同じ画像でID 0・ID 1の双方の有効な姿勢を得て、base_offset_calibratedがtrueの場合のみ**配信。frame_idはbase_link、単位m、姿勢Quaternion。

```bash
ros2 topic echo /aruco/object/pose
```

座標変換は `base_T_marker0 × inverse(camera_T_marker0) × camera_T_marker1 × marker1_T_object`。

TFは `base_link → camera_optical_frame → aruco_marker_1 → aruco_object_origin` とつながる。ID 0もカメラの子のままにし、`base_link → ID 0` を重複配信しない。カメラにすでに別のTF親・配信元がある場合は `publish_camera_tf: false` にし、どちらを正しいカメラ外部校正として使うか揃える。ID 0が見えない場合はbase→cameraを更新しない。RViz/tf2には古い値が残り得るため、未検出時の表示を新しい測定値だと解釈しない。

アームへの動作指令は送らない。20 mmの単一平面マーカーの姿勢誤差が基準座標にも伝わるため、ID 0の姿勢反転・揺れがある場合はオフセット値で無理に相殺せず、撮影条件や基準ボードを改善する。

## SO-101と推定原点・20 mm物体を同時表示

校正済みカメラを起動したまま、既存のArUco・RViz・重複するrobot_state_publisherを停止してから実行する。旧表示ノードのcamera→worldの静的TFは廃止したため、古い起動プロセスとRVizは必ず終了する。

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 launch so101_aruco preview.launch.py offsets_file:=$HOME/robot_arm/src/so101_aruco/config/marker_offsets.yaml
```

RVizのFixed Frameは **world**。TFは次の関係になる。

```text
world → base_link → camera_link → camera_optical_frame → aruco_marker_0 → base_link_predict
                                       → aruco_marker_1 → aruco_object_origin
```

world→base_linkはロボットURDFの固定関係。現URDFではゼロ変換だが、この計算はworldとbase_linkが同じ原点であることを要求しない。base_link→カメラはID 0を有効に検出するたびに更新する。物体Poseは同一画像でID 0・1の両方が有効な場合だけ更新する。カメラを動かすと動くのはカメラTFで、静止したロボットモデルは動かない。固定した物体のbase_link基準座標も理想的には変わらない。

`marker_offsets.yaml` の `base_offset_calibrated` をtrueにして現在のオフセットによる変換を有効化済み。値の実測精度を保証するものではない。設定したID 0の位置・姿勢が誤っている場合や単一マーカーの姿勢が揺れる場合は、カメラ移動に伴う物体位置のずれが残り得る。

水色の立方体は `/aruco/object/pose` のbase_link基準Poseに配置し、中心が物体原点、一辺20 mm。両マーカーが同時に見えなくなったら更新せず、0.5秒で消える。rawマーカーTFには履歴が残るため、未検出時の位置を最新測定と扱わない。

モデルは既定で表示専用の仮の関節値を使用し、モーターへ指令を送らない。実機joint_statesが既にある場合は `use_fake_joints:=false joint_states_topic:=/so101_follower/joint_states` 等を実際の配信名に合わせて追加する。既存の検出ノードを使う場合は `start_detector:=false` とし、その検出側で `base_offset_calibrated: true` と `publish_camera_tf: true` が必要。

`preview.yaml` は立方体のサイズだけを設定する。以前の `robot_base_in_camera` によるカメラ基準のモデル配置は使用しない。`base_link_predict` はID 0と同じオフセットから逆算しているため、有効化後はbase_linkと一致する。独立した校正精度の検証には既知の物理寸法・位置との比較が必要。

### RVizの操作と一時的な未検出

ツールバーのMove Cameraで左ドラッグが回転、中央ドラッグ（またはShift＋左ドラッグ）が平行移動、ホイールが拡大縮小。ViewsパネルのOrbitはworldを中心座標系に使用する。操作できない場合はMove Cameraを選び直す。

TF表示は最終更新から5秒で非表示にする設定。これは表示上の保持であり、未検出中に新しい計測値としてTFを再配信するものではない。ID 0が見えていればID 1が一時的に見えなくてもカメラTFは更新する。ID 0が隠れた場合はworldへの接続が更新されず、カメラを動かした際の補正もできない。物体Poseは引き続き両IDを同時に検出した画像に限る。

## カメラ本体座標と光学座標

`base_link → camera_link → camera_optical_frame` の順に配信する。camera_linkはX前方・Y左・Z上、camera_optical_frameはX画像右・Y画像下・Zレンズ前方。原点は同じ光学中心で、筐体中心への並進は加えていない。camera_linkの＋Xがレンズ前方を向く。

両座標の固定回転は `/tf_static`、ID 0から求めるbase_link→camera_linkは `/tf` に配信する。既存の光学座標でのマーカー検出と物体Poseの数値は変わらない。camera_linkはID 0の有効な観測後に表示される。カメラのTFを他ノードが配信している場合は重複させない。

変更後は古いArUcoノードとRVizを終了してpreview.launch.pyを再起動する。RVizの古いTF履歴を残さず、新しい親子関係を読み込むため。

## 静止時の姿勢ジャンプ

実測15秒では、ID 0の画像上の位置がほぼ静止でも推定姿勢のフレーム間変化が最大18.7度となり、カメラ位置の変動幅が最大79.5 mm、物体位置の変動幅が最大46.0 mmになった。ID 1のカメラ基準位置の変動幅は1 mm未満。TF配信元の重複は見つからなかった。記録は `log/camera_analysis/aruco_jitter.json`。

平面マーカーの複数姿勢候補の切替が疑われるため、IPPEの候補を取得し、再投影誤差が最小値から `pose_ambiguity_px`（既定0.3 px）以内の候補間では直前の姿勢に近いものを選択する。過去の観測が `pose_history_timeout_s`（既定0.5秒）より古い場合は履歴を使わない。平均化や未検出時の再配信はしない。両値は `marker_offsets.yaml` で調整可能。再投影誤差の差が大きいときは新しい最小誤差解を選ぶため、姿勢ジャンプを完全に禁止する処理ではない。

最初に選択された候補が正しいとは保証できず、20 mmマーカー自体の姿勢・位置ノイズも残る。改善不足なら撮影距離・照明・斜めからの観測、複数マーカーの基準ボードなどを検討する。回転オフセットを変更してノイズを相殺しない。
