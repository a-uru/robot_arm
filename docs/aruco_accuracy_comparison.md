# CRANE-X7実装を参考にしたArUco推定改善

参照リポジトリ：[rt-net/crane_x7_ros ros2](https://github.com/rt-net/crane_x7_ros/tree/ros2)。確認コミット：`f30e8ff82314c6678f794bac42ccec207faf7d61`。

参照した実装：

- [Python版aruco_detection.py](https://github.com/rt-net/crane_x7_ros/blob/f30e8ff82314c6678f794bac42ccec207faf7d61/crane_x7_examples_py/crane_x7_examples_py/aruco_detection.py)
- [C++版aruco_detection.cpp](https://github.com/rt-net/crane_x7_ros/blob/f30e8ff82314c6678f794bac42ccec207faf7d61/crane_x7_examples/src/aruco_detection.cpp)

## 実装・条件の違い

CRANE-X7側は画像・CameraInfoを同期し、OpenCVの `estimatePoseSingleMarkers` により位置・姿勢を求めてTFを配信する。特別な平滑化はない。コード上は `DICT_6X6_50`、黒枠辺長0.04 m。READMEでは一辺50 mmの立方体に印刷マーカーを取り付ける説明になっている。立方体サイズとコードのマーカー辺長を混同しない。

現在の構成は `DICT_4X4_50`・黒枠20 mm。印刷済みマーカーと寸法は維持し、辞書や辺長をCRANE-X7へ合わせて変更していない。カメラの解像度・校正も維持。

## 取り込んだ改善

1. 画像とCameraInfoをTimeSynchronizerで同時刻の組にする（参照実装と同じ考え方）。内部パラメータが不変なら、この変更だけで静止時の姿勢揺れが減るわけではない。
2. OpenCV旧API・新APIの両方でサブピクセル補正を明示する。従来は新API分岐に補正指定がなく、旧API側も既定精度に依存していた。窓半径5 px、最大50反復、終了精度0.001 pxとする。
3. IPPEの候補と履歴による選択を維持し、選択後に `solvePnPRefineLM` で反復最適化する。再投影誤差が悪化する解、非有限値、負の奥行きは採用しない。参照実装と同系統の反復推定も比較したが、別の撮影条件での候補切替に備えてIPPEの候補選択は残した。
4. 時間平均による平滑化は追加していない。未検出時に古い姿勢を再配信する処理も追加していない。

調整値は `src/so101_aruco/config/marker_offsets.yaml` の `corner_refinement_window`、`corner_refinement_iterations`、`corner_refinement_accuracy`、`pose_refine_lm`。通常は既定値で使用する。窓を大きくしすぎると小さなマーカーの内部模様を含み、かえって補正を損なう可能性がある。

## 同一画像での比較

ユーザーがカメラと両マーカーを静止させた状態で約12秒・179フレームの四隅と内部パラメータのみ記録。画像は保存していない。各フレームで現実装の検出結果、補正なし、厳密な3/5 px補正を計算し、同じ観測から各推定法をオフライン比較した。

| 指標 | 変更前（現在の検出設定＋IPPE履歴） | 厳密5 px補正＋IPPE履歴＋LM |
| --- | ---: | ---: |
| 再投影RMS誤差の平均 | 0.2446 px | 0.2270 px |
| ID 0の最大フレーム間姿勢差 | 2.652度 | 2.168度 |
| ID 1の最大フレーム間姿勢差 | 2.739度 | 2.328度 |
| ID 0から逆算したカメラ位置の最大軸方向変動幅 | 14.30 mm | 11.72 mm |
| ID 0基準の物体位置の軸別変動幅 | 6.98 / 6.29 / 4.91 mm | 5.73 / 5.75 / 4.44 mm |
| 同位置の軸別標準偏差 | 1.48 / 1.06 / 0.98 mm | 1.27 / 1.03 / 0.89 mm |

物体変動は `inverse(camera_T_marker0) × camera_T_marker1` の並進、つまりID 0座標系で比較。base_linkへの固定回転を適用すると軸別成分は変わる。初期実験の「補正なし」と比較すると大きく改善したが、変更前の実装も補正を行っていたため、それを変更前の性能として報告しない。

比較条件では参照実装と同系統の反復推定（SOLVEPNP_ITERATIVE）とIPPE選択後のLMがほぼ同じ結果となった。改善は確認できたが、数mmの変動は残っており、今回の変更だけで揺れがなくなったとは言えない。実際の物体位置の真値は測っていないため、示したものは再投影誤差と静止時の再現性であり、絶対位置精度の保証ではない。

データ：`log/camera_analysis/aruco_corner_benchmark.json`、`aruco_solver_comparison.json`（Git管理外）。

## 起動と限界

カメラはそのまま、既存のArUcoプレビュー起動をCtrl+Cで停止し、再起動する。

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch so101_aruco preview.launch.py
```

同期後は画像とCameraInfoの時刻が一致しない配信元では処理されない。現在のUSBドライバは両方に同じヘッダーを設定している。

20 mmの単一平面マーカーから傾きを推定し、その逆変換でカメラを定位する構成は、回転誤差が位置へ増幅される。CRANE-X7の実装へ単に置き換えても、この幾何学的な条件は変わらない。さらに改善が必要なら、ID 0を大きくする・複数マーカーを既知配置した基準ボードにする・固定カメラなら外部位置姿勢を校正して固定する、などが候補。これらは今回自動では変更していない。
