# オリジナルロボットへの適用

この文書では、`crawler_mujoco` の車体寸法、履帯、フリッパ、センサ、ROS 2
インターフェースを独自ロボットに合わせる手順を説明します。

`crawler_mujoco` は URDF や SDF のロボットモデルをそのまま読み込む方式ではなく、
`config/robot.yaml` の値から MuJoCo の MJCF を生成します。まず設定値で形状を合わせ、
必要な場合だけ mesh や機構生成コードを変更してください。

## 対応範囲

設定変更だけで対応できる標準構成は次のとおりです。

- 左右2本の主履帯
- 各履帯の前後に1個ずつ、合計4個のスプロケット
- 前後左右に1枚ずつ、合計4枚のフリッパ
- skid steering による並進・旋回
- 車体固定の IMU
- 前後各1台の RGB-D カメラ

車輪数、履帯数、フリッパ数、joint 軸、リンク階層が異なる場合は、
[機構構成を変更する場合](#機構構成を変更する場合)も参照してください。

> [!IMPORTANT]
> このパッケージはシミュレータです。実機モータへ直接指令を送るドライバではありません。
> 実機と同じ ROS topic を接続する場合も、速度・角度・トルク制限、非常停止、通信断時停止は
> 実機側の安全制御で実装してください。

## 1. ロボットを計測する

座標系は REP 103 に合わせ、`base_link` の前方を `+x`、左を `+y`、上を `+z` とします。
長さは m、質量は kg、角度は rad、力は N、トルクは N·m です。

最低限、次の値を実機図面または CAD から取得します。

| 対象 | 必要な値 | 設定キー |
| --- | --- | --- |
| 車体 | 質量、全長、全幅、全高 | `chassis_mass`, `chassis_size` |
| 主履帯 | 左右履帯中心間距離、履帯幅 | `track_separation`, `track_width` |
| スプロケット | 前後軸間距離、半径、1個あたりの質量 | `wheelbase`, `wheel_radius`, `wheel_mass` |
| 履帯外形 | 履帯中心線半径、接触面までの有効半径 | `track_radius`, `track_contact_radius` |
| ベルト | 厚さ、直線部の近似質量 | `belt_thickness`, `straight_section_mass` |
| フリッパ | hingeから先端軸までの長さ、幅、質量、左右取付位置 | `flipper_length`, `flipper_width`, `flipper_mass`, `flipper_y_offset` |
| 可動範囲 | フリッパ下限・上限 | `flipper_lower`, `flipper_upper` |
| 駆動系 | 最大速度、スプロケット最大トルク、フリッパ最大トルク | `max_commanded_*`, `drive_force`, `flipper_effort` |
| センサ | IMUとカメラの取付位置 | `imu_position`, `front_position`, `rear_position` |

似ている値でも、次のキーは用途が異なります。

- `track_separation`: MuJoCo上の左右履帯中心の幾何学的な間隔
- `drive_track_width`: 差動速度 `v_left/right = linear ∓ width × angular / 2` に使う有効幅
- `wheel_radius`: `/cmd_vel` からスプロケット角速度へ変換する半径
- `track_radius`: スプロケットと履帯形状を配置する半径
- `track_contact_radius`: ベルト表面速度と接線力の換算に使う有効半径

旋回時の実効幅は横滑りの影響を受けるため、`drive_track_width` は図面値を初期値にして、
実機の旋回角速度と一致するよう最後に校正します。

## 2. 独自設定ファイルを作る

既定値を直接上書きせず、まず完全な設定をコピーします。

```bash
cp config/robot.yaml config/my_robot.yaml
cp config/default.yaml config/my_robot_sim.yaml
```

`config/my_robot_sim.yaml` の参照先を変更します。

```yaml
robot_config: my_robot.yaml
```

以降は `config/my_robot.yaml` の値を実機に合わせます。設定漏れを避けるため、不要に見える
キーも最初は削除しないでください。現在のジェネレータでは、一部のキーが必須です。

### 車体と主履帯

```yaml
robot:
  spawn_x: 0.0
  spawn_y: 0.0
  spawn_z: 0.45

  chassis_mass: 42.0
  chassis_size: [0.62, 0.34, 0.12]
  imu_position: [0.02, 0.0, 0.08]

  track_separation: 0.39
  drive_track_width: 0.39
  track_width: 0.11
  wheelbase: 0.50
  wheel_radius: 0.085
  belt_thickness: 0.018
  track_radius: 0.105
  track_contact_radius: 0.114
  wheel_mass: 0.45
  straight_section_mass: 0.12
  straight_enabled: true
```

`chassis_size` は半寸法ではなく `[全長, 全幅, 全高]` です。`spawn_z` は、初期状態で
履帯と地面が深く重ならず、数 cm 程度落下して接地する高さから調整します。

### フリッパ

```yaml
robot:
  flipper_enabled: true
  flipper_length: 0.31
  flipper_width: 0.045
  flipper_y_offset: 0.245
  flipper_mass: 3.0
  flipper_lower: -1.57079632679
  flipper_upper: 1.57079632679
  flipper_effort: 180.0
  flipper_max_velocity: 0.7
  flipper_track_drive: true
```

フリッパの原点は主履帯の前後スプロケット軸です。`flipper_length` は hinge から先端
スプロケット軸まで、`flipper_y_offset` は `base_link` 中心から hinge までの y 距離です。
全jointで、正の角度はフリッパ先端を持ち上げる向きです。

現在の質量モデルでは、1枚のフリッパに0.2 kgの近似部品を4個配置します。
そのため `flipper_mass` は **0.8 kgより大きい値**にしてください。

フリッパのない車体は、単体実行では `flipper_enabled: false` にできます。ROS 2連携では
4 jointを前提とするtopicと可視化モデルも変更する必要があります。

### グローサと接触

`config/my_robot_sim.yaml` の `grouser` を変更します。

```yaml
grouser:
  shape: rectangle
  elements_per_round: 44
  width: 0.018
  height: 0.014
  friction: [2.0, 0.03, 0.003]
  longitudinal_friction: 2.0
  lateral_friction: 0.45
  solref: [0.008, 1.0]
  solimp: [0.95, 0.99, 0.001]
```

`elements_per_round` は主履帯外周からグローサ間隔を決め、その間隔をフリッパにも使用します。
最初は `shape: none` で車体寸法と駆動方向を確認し、次に `rectangle`、必要なら
`semicircle` または `spike` を試してください。

摩擦係数と `solref` / `solimp` は、寸法・質量・駆動速度が確定してから調整します。
摩擦を先に過度に上げると、モデル誤差を隠して不自然な旋回や振動を生むことがあります。

### 速度と駆動トルク

```yaml
robot:
  commanded_linear_speed: 0.25
  max_commanded_linear_speed: 0.50
  max_commanded_angular_speed: 1.5
  command_scale: 1.0
  track_max_acceleration: 0.8
  drive_force: 700.0
  velocity_servo_strength: 0.05
  yaw_servo_strength: 0.0
```

調整は次の順序を推奨します。

1. `command_scale` で直進速度を実機へ合わせる。
2. `drive_track_width` で旋回角速度を合わせる。
3. `track_max_acceleration` で加減速を合わせる。
4. `drive_force` と `velocity_servo_strength` で停止時・登坂時の駆動トルクを合わせる。
5. 必要な場合だけ `yaw_servo_strength` を少しずつ上げる。

`yaw_servo_strength` は旋回を補助するMuJoCo上の仮想servoです。実機再現を優先する場合は
`0.0`から開始し、履帯接触だけでは実機の旋回を再現できない場合に限って使用してください。

### フリッパPID

```yaml
robot:
  flipper_position_kp: 900.0
  flipper_position_ki: 0.0
  flipper_position_kd: 25.0
  flipper_position_integral_limit: 0.5
```

まず `ki: 0.0` で `kp` と `kd` を調整し、負荷時の定常偏差が問題になる場合だけ `ki` を
追加します。振動する場合は `kp`、`flipper_max_velocity`、またはsimulationの`timestep`を
下げて確認してください。積分項が飽和する場合は `flipper_position_integral_limit` を小さくします。

### センサ位置

```yaml
sensors:
  rgbd:
    enabled: true
    front_position: [0.31, 0.0, 0.10]
    rear_position: [-0.31, 0.0, 0.10]
    downward_pitch_deg: 18.0
```

IMU位置は `robot.imu_position`、カメラ位置は `base_link` 基準です。カメラ姿勢は前後向きと
共通の下向きpitchを前提としているため、roll、yaw、左右非対称配置が必要な場合は
`crawler_mujoco/simulator.py` と `crawler_mujoco/robot_description.py` の両方を変更します。

## 3. MJCFを生成して確認する

まずシミュレーションを走らせず、生成されたMJCFを確認します。

```bash
./.venv/bin/python scripts/crawler_mujoco \
  --config config/my_robot_sim.yaml \
  --shape none \
  --write-xml-only \
  --output /tmp/my-robot-mjcf
```

次に平坦な内蔵環境でGUIを起動します。

```bash
./.venv/bin/python scripts/crawler_mujoco \
  --config config/my_robot_sim.yaml \
  --shape none \
  --gui --show-contacts
```

以下を順番に確認してください。

1. 初期状態で車体と地面が重なっていない。
2. 左右の履帯位置、スプロケット軸間、フリッパhingeが図面と一致する。
3. 正の直進指令で `+x` へ進む。
4. 正の角速度指令で反時計回りに旋回する。
5. 4枚のフリッパが、左前・左後・右前・右後の順で正方向へ上がる。
6. `rectangle` などのグローサを有効にしても初期貫通や激しい振動がない。

形状の確認後にフリッパ追従を検証できます。

```bash
./.venv/bin/python -m tools.validate_flipper_control \
  --config config/my_robot_sim.yaml \
  --output /tmp/my-robot-flipper.csv
```

## 4. ROS 2で接続する

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select crawler_mujoco
source install/setup.bash

ros2 launch crawler_mujoco crawler_mujoco.launch.py \
  config:=/absolute/path/to/config/my_robot_sim.yaml \
  python_executable:=/absolute/path/to/crawler_mujoco/.venv/bin/python \
  shape:=rectangle
```

### 固定されている名前と順序

標準構成では、次の名前がコード上の契約です。

| 種類 | 名前・順序 |
| --- | --- |
| base frame | `base_link` |
| IMU frame | `imu_link` |
| 前後カメラ | `front_rgbd_optical_frame`, `rear_rgbd_optical_frame` |
| フリッパjoint | `joint_left_front`, `joint_left_rear`, `joint_right_front`, `joint_right_rear` |
| フリッパ配列 | 左前、左後、右前、右後 |
| 速度入力 | `/cmd_vel`, `/target/cmd_vel` |
| フリッパ入力 | `/crawler/flipper_commands`, `/target/joint_states` |
| 状態出力 | `/joint_states`, `/odom`, `/imu/data`, `/clock` |
| TF | `map -> base_link`、`base_link`以下は`robot_state_publisher` |

接続確認の例:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'

ros2 topic pub --once /crawler/flipper_commands std_msgs/msg/Float64MultiArray \
  '{data: [0.2, 0.2, 0.2, 0.2]}'
```

既存ロボット側のtopic名が異なる場合は、まずROSのremapを使います。メッセージ型、joint数、
角度符号が異なる場合は、実機ドライバを直接変更せず、小さな変換nodeを間に置くと安全です。

## CAD meshを使う場合

現在の車体と履帯は box / cylinder のプリミティブで生成します。外観や重心形状を実機へ
近づける場合は、次の方針でmeshを追加します。

1. meshをm単位・右手座標系へ変換する。
2. 可視meshと接触用meshを分ける。
3. 接触用meshは凸形状または凸分解した複数meshにする。
4. `assets/models/<robot_name>/` に配置する。
5. `crawler_mujoco/simulator.py` の `build_xml()` で `<asset>` と `<geom>` を追加する。
6. RVizも同じ外観にする場合は `crawler_mujoco/robot_description.py` を更新する。

高密度なCAD meshをそのままcollisionへ使うと、接触が不安定になり計算時間も増えます。
visualは詳細mesh、collisionは単純なbox/cylinderまたは凸分解meshにすることを推奨します。

## 機構構成を変更する場合

標準の2履帯・4フリッパ構成から外れる場合は、以下を一組として変更します。

| ファイル・関数 | 変更内容 |
| --- | --- |
| `crawler_mujoco/simulator.py:build_xml()` | 車体、主履帯、sensor、actuatorのMJCF生成 |
| `crawler_mujoco/simulator.py:straight_belt_xml()` | 主履帯直線部の構造 |
| `crawler_mujoco/simulator.py:flippers_xml()` | フリッパbody、joint、actuator |
| `crawler_mujoco/simulator.py:set_velocity_controls()` | `/cmd_vel`から各履帯速度への変換 |
| `crawler_mujoco/simulator.py:set_flipper_controls()` | 指令配列とjointの対応 |
| `crawler_mujoco/simulator.py:update_flipper_controls()` | フリッパPIDとactuator名 |
| `crawler_mujoco/ros_bridge.py:FLIPPER_JOINTS` | ROS上のjoint名と順序 |
| `crawler_mujoco/ros_bridge.py:publish_state()` | JointState、Odometry、IMU、TF |
| `crawler_mujoco/robot_description.py` | RViz用URDFのリンク・joint構成 |
| `launch/crawler_mujoco.launch.py` | node、remap、robot_description |

MuJoCo側だけを変更すると、RVizの形状やJointStateが一致しなくなります。MJCF、URDF、ROS
メッセージのjoint名・frame名を同時に揃えてください。

### 慣性について

現状は `chassis_mass` などの質量を設定できますが、車体、スプロケット、フリッパの
`diaginertia` の一部は `simulator.py` 内の固定近似値です。実機の姿勢応答を評価する場合は、
CADから重心位置と慣性テンソルを取得し、各 `<inertial>` の `pos`、`diaginertia`、必要なら
`quat` を実機値へ変更してください。外形と総質量だけを合わせても、転倒・pitch応答は一致しません。

## 推奨する調整順序

1. 座標系、外形寸法、joint位置
2. collision形状と初期接地
3. 質量、重心、慣性
4. 直進速度と旋回速度
5. 加速度、駆動トルク、横滑り
6. フリッパ可動範囲とPID
7. グローサ形状と接触パラメータ
8. IMU、カメラ、ROS frame
9. 実機ログとの比較

一度に複数分類を変更せず、平地直進、定常旋回、単独フリッパ、段差の順に条件を増やすと、
差の原因を切り分けやすくなります。

## 完了チェックリスト

- [ ] `--write-xml-only` でMJCFを生成できる
- [ ] GUI起動直後に貫通、跳ね、発散がない
- [ ] 車体寸法、履帯位置、フリッパ軸が図面と一致する
- [ ] 直進・旋回の向きと速度が実機に一致する
- [ ] フリッパの順序、符号、可動範囲、速度、保持力が一致する
- [ ] 質量、重心、慣性を確認した
- [ ] 接触条件を変えても数値計算が安定する
- [ ] `/joint_states`、`/odom`、`/imu/data`、TFのframeが一致する
- [ ] カメラ位置・向きとCameraInfoが一致する
- [ ] 実機ドライバとの間に速度・トルク制限と非常停止がある

すべてを満たした後に、実機と同じ速度指令・地形条件のログを取得し、位置、姿勢、
フリッパ角、到達時間を比較してください。
