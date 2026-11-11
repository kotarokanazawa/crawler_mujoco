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

## sensors.mid360

Mid-360を点群LiDARと内蔵6軸IMUからなる複合センサとして設定します。
`frames`へ1台追加するだけで、同じ位置・姿勢・`frame_id`を持つMuJoCo raycast
siteとIMU siteが生成されます。

```yaml
sensors:
  mid360:
    enabled: true
    lidar:
      output_format: pointcloud2
      publish_rate: 10.0
      points_per_scan: 2000
      pattern_file: package://crawler_mujoco/assets/lidar_patterns/livox_mid360_pattern/livox_mid360_measured.yaml
    imu:
      noise:
        enabled: true
    frames:
      - name: front
        frame_id: front_lidar_link
        pointcloud_topic: /livox/front/lidar
        custom_topic: /livox/front/custom
        imu_topic: /livox/front/imu
        position: [0.275, 0.0, 0.065]
        rpy_deg: [0.0, 20.0, 0.0]
```

各frameでは次を個別指定できます。

- `name`: MuJoCo内部site/sensor名の接頭辞
- `frame_id`: PointCloud2とImuの共通ROS frame
- `pointcloud_topic`, `custom_topic`, `imu_topic`: それぞれの配信先
- `position`, `rpy_deg`: `base_link`基準の共通取付姿勢
- `lidar_site_name`, `imu_site_name`, `imu_sensor_name`: 必要な場合のみ内部名を上書き
- `imu_aliases`: 同じIMUを配信する追加topic
- `lidar`, `imu`: その1台だけに適用する設定上書き

従来の`sensors.lidar`と`sensors.imu`も使用でき、Mid-360 frameと併設できます。

## sensors.imu

`frames`ごとにMuJoCoの6軸IMU（3軸角速度＋3軸並進加速度）を生成します。
各エントリでは`site_name`、`sensor_name`、ROSの`frame_id`と`topic`、
`base_link`基準の`position`、`rpy_deg`を設定します。`aliases`を指定すると同じ
メッセージを互換topicにも配信します。

6軸センサのため絶対姿勢は計測値として配信せず、`sensor_msgs/Imu`の
`orientation_covariance[0]`を`-1`に設定します。角速度と加速度は各`frame_id`
のローカル軸で表現されます。既定設定では車体IMUに加えて、前後Livoxの
`front_lidar_link`、`rear_lidar_link`基準でも配信します。

`noise`ではICM-40609相当の確率モデルを設定します。

- `seed`: 再現可能な乱数seed。各frameは`seed + frame index`を使用
- `sample_rate_hz`: IMUの独立配信周波数、および白色ノイズ密度を
  1サンプル標準偏差へ変換する周波数。既定値は200 Hz
- `gyro_noise_density_dps_sqrt_hz`, `accel_noise_density_g_sqrt_hz`: 白色ノイズ密度
- `*_initial_bias_std_*`: 起動時に個体ごとに固定されるバイアスの標準偏差
- `*_bias_random_walk_*`: 時間とともに変動するバイアスランダムウォーク
- `*_scale_error`: 軸ごとの感度誤差上限
- `gyro_full_scale_dps`, `accel_full_scale_g`: 飽和範囲
- `quantization_enabled`, `adc_levels`: ADC量子化

既定の白色ノイズ密度、感度誤差、フルスケールはICM-40609の公開仕様を基準にし、
初期バイアスとランダムウォークはシミュレーション調整値です。`noise.enabled: false`
にするとMuJoCo真値をそのまま配信します。frameごとの`noise`マッピングで共通設定を
上書きできます。

## sensors.lidar

`enabled: true` にすると、MuJoCo の collision geom に対する raycast で Livox
Mid-360 風の非反復走査を生成し、ROS 2 の `/livox/lidar` へ
`sensor_msgs/PointCloud2` または `livox_ros_driver2/CustomMsg` を配信します。
`frames` に複数の取付設定を記述できます。

- `output_format`: `pointcloud2`、`custom`、または両方を出す`both`
- `publish_rate`: 点群配信周波数 [Hz]
- `points_per_scan`: 1 メッセージあたりの ray 数（大きいほど計算負荷が増加）
- `min_range`, `max_range`: 有効距離 [m]
- `vertical_min_deg`, `vertical_max_deg`: 垂直視野角 [deg]
- `frames[].site_name`: MuJoCo内部で使用する一意なsite名
- `frames[].frame_id`: 点群ヘッダとTFで使用するROS frame名
- `frames[].topic`: 点群の配信topic
- `frames[].custom_topic`: Livox CustomMsgの配信topic
- `frames[].position`: `base_link` 基準の取付位置 [m]
- `frames[].rpy_deg`: `base_link` 基準の roll、pitch、yaw [deg]

実機固有の発射タイミング表ではなく、Mid-360 の 360° × 59° 視野を再現する
決定論的な近似パターンです。現在のraycastは全点を同一のMuJoCo姿勢で計算する
瞬時スキャンなので、CustomMsgの全点を`offset_time=0`として配信します。
各点には0～3の`line`、`tag=0`、設定可能な`reflectivity`（既定値100）も
格納します。センサ移動によるスキャン歪み自体は再現しません。

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
