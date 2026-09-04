# 設定リファレンス

`config/default.yaml` は実行条件、`config/robot.yaml` はロボット固有値を保持します。
`robot_config` は実行条件ファイルからの相対パスで解決され、同じファイル内の `robot` 項目が
ある場合は外部ファイルの値を上書きします。長さは m、角度は特記がなければ rad です。

## simulation

| キー | 説明 |
| --- | --- |
| `timestep` | 物理計算刻み [s] |
| `duration` | UI/ROS なしで実行する時間 [s] |
| `render_fps` | CSV 記録と ROS 配信の基準周波数 [Hz] |
| `show_contacts` | GUI で接触点・接触力を初期表示 |
| `integrator` | MuJoCo integrator 名 |

## sensors.rgbd

`enabled` が `false` の場合、カメラをモデルへ追加せず、レンダラも生成しません。
`width`、`height`、`publish_rate`、`fovy_deg`、`downward_pitch_deg` で画像を設定します。
`front_position` と `rear_position` は `base_link` 基準の取付位置です。
`pointcloud_stride` を大きくすると点群帯域を削減できます。

## grouser

- `shape`: `none`、`rectangle`、`semicircle`、`spike`
- `elements_per_round`: 主履帯外周を基準とするグローサ数
- `width`、`height`: 進行方向の幅と突出高さ
- `friction`: MuJoCo の sliding、torsional、rolling friction
- `longitudinal_friction`、`lateral_friction`: 履帯前後・横方向の異方性摩擦
- `solref`、`solimp`: 接触拘束パラメータ

## environment

`arena_yaml: ""` は依存のない内蔵 bridge を使用します。相対パスを指定した場合は
`assets/arenas/`、`model://` URI は `assets/models/` から解決します。

`friction` は地面と arena の摩擦、`arena_ground_z` と `builtin_ground_z` は地面高さです。
`bridge_start_x`、`bridge_end_x`、`bridge_width`、`bridge_thickness` は内蔵 bridge と
評価区間を定義し、`target_x` は到達判定位置です。

## robot

| 分類 | 主なキー |
| --- | --- |
| 初期姿勢 | `spawn_x`, `spawn_y`, `spawn_z` |
| 車体 | `chassis_mass`, `chassis_size`, `imu_position` |
| 主履帯 | `track_separation`, `drive_track_width`, `track_width`, `wheelbase`, `wheel_radius`, `track_radius` |
| フリッパ | `flipper_enabled`, `flipper_length`, `flipper_width`, `flipper_y_offset`, `flipper_lower`, `flipper_upper` |
| 速度制御 | `commanded_linear_speed`, `max_commanded_linear_speed`, `max_commanded_angular_speed`, `command_scale` |
| 駆動調整 | `track_max_acceleration`, `drive_force`, `velocity_servo_strength`, `yaw_servo_strength` |
| フリッパ PID | `flipper_effort`, `flipper_max_velocity`, `flipper_position_kp`, `flipper_position_ki`, `flipper_position_kd`, `flipper_position_integral_limit` |

`track_max_acceleration` を上げると応答は速くなりますが、停止時の反力も増えます。
振動が出る場合は、まずこの値か `velocity_servo_strength` を小さく調整してください。
`drive_track_width` は差動速度計算に使う有効幅、`track_separation` はモデルの左右履帯中心間です。
