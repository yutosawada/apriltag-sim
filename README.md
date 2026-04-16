# apriltag_sim

`apriltag-detector` と同じ契約（`camera_optical_frame → tag_name` の TF）を、
画像処理を介さずに合成するシミュレータ。`lidar_sim` と同じ設計思想で、
`/sim_truth/pose` と YAML に書かれたタグ配置だけから「カメラ光学系 → タグ」
の TF を発行する。

下流の `amr_api::_getHomeTags`, `cart_detect`, `tf_location` は TF を名前パターンで
検索しているだけなので、画像合成なしでそのまま動く。

## 仕組み

1. `/sim_truth/pose` から `map → base_footprint`（真値）を取得
2. `base_footprint → <camera>_realsense_color_optical_frame` を URDF から tf2 lookup（静的なのでキャッシュ）
3. YAML に書かれた各タグに対し、全カメラで可視判定
   - FOV（水平/垂直）・最小/最大距離
   - 背面カリング（カメラがタグの表側にあるか）
4. 最も近いカメラから `optical_frame → tag_name` の TF を発行

同一タグを複数のカメラが同時に見られるケースでは、TF ツリーの一意性を保つため
「最も近いカメラ」のみが publish する。これは実機の挙動にも近い
（通常、距離が近いカメラの方が検出安定）。

## パラメータ

`config/apriltag_sim.yaml` 参照。

| キー | 既定 | 説明 |
|------|------|------|
| `pose_topic` | `/sim_truth/pose` | 真値ポーズの入力トピック |
| `base_frame` | `base_footprint` | ロボット基準フレーム |
| `publish_rate_hz` | 20.0 | TF 発行レート |
| `cameras.names` | `[front, mid, rear]` | 有効なカメラのリスト |
| `cameras.<name>.optical_frame` | `<name>_realsense_color_optical_frame` | 親フレーム名 |
| `cameras.<name>.hfov_deg` | 69.4 | 水平視野角 |
| `cameras.<name>.vfov_deg` | 42.5 | 垂直視野角 |
| `cameras.<name>.max_range` | 3.0 | 検出可能最大距離 [m] |
| `cameras.<name>.min_range` | 0.1 | 検出可能最小距離 [m] |
| `tag_names` | `[]` | タグのフレーム名（`home_11` など） |
| `tag_xs/ys/zs` | `[]` | タグ位置（map 座標系） |
| `tag_yaws_face` | `[]` | タグが向いている方角 [rad]（+z がこの向き） |

## タグ座標系の約束

AprilTag の標準検出姿勢に合わせる。

- `+z`: タグ表面から観察者方向（壁に貼ったタグなら部屋側へ）
- `+y`: ワールドの上方向
- `+x`: `y × z`（タグを正面から見て右）

`yaw_face` は map 座標系で `+z` がどの方角を向くかをラジアンで指定する。

## 単独起動

```bash
ros2 launch apriltag_sim apriltag_sim.launch.py
```

または別 YAML で上書き。

```bash
ros2 launch apriltag_sim apriltag_sim.launch.py \
  config_file:=/path/to/custom.yaml
```

## sim_amr_bringup への組み込み

`sim_sensors.launch.py` から取り込まれる。`with_apriltags:=true` で有効化
（既定は false。タグ配置 YAML を用意してから有効化する）。

## 動作確認

```bash
# ノード状態
ros2 node list | grep apriltag_sim

# タグ TF が出ているか
ros2 run tf2_ros tf2_echo mid_realsense_color_optical_frame home_11

# amr_api が見られる状態か（base_footprint -> home_11 が通るか）
ros2 run tf2_ros tf2_echo base_footprint home_11
```

## Phase 1 の制約

- 遮蔽（他の物体でタグが隠れる）は考慮しない。単純な FOV＋距離＋背面判定のみ
- ノイズ（位置ジッタ・検出失敗）は未実装
- `AprilTagDetectionArray` メッセージは未発行（TF のみ消費する下流に対応）
