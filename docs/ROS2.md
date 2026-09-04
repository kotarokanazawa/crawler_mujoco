# ROS 2 連携

## ビルド

ROS 2 Humble を source したワークスペースでビルドします。

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select crawler_mujoco
source install/setup.bash
```

MuJoCo は ROS の Python と別の仮想環境に置けます。

```bash
ros2 launch crawler_mujoco crawler_mujoco.launch.py \
  python_executable:=/path/to/crawler_mujoco/.venv/bin/python \
  shape:=rectangle
```

同梱 arena を使う場合は `arena_yaml:=benchmark.yaml` を追加してください。

GUI群を起動せず、ROSブリッジ付きシミュレータだけを実行する場合:

```bash
source /path/to/crawler_mujoco/.venv/bin/activate
ros2 run crawler_mujoco crawler_mujoco --shape rectangle --ros2
```

## Launch 引数

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `config` | package の `config/default.yaml` | 設定ファイル |
| `python_executable` | source `.venv` または現在の Python | MuJoCo 実行 Python |
| `output_directory` | `~/.ros/crawler_mujoco/results` | CSV/MJCF 出力先 |
| `shape` | `semicircle` | グローサ形状 |
| `arena_yaml` | 設定ファイル値 | 同梱 arena YAML |
| `mujoco_gui` | `true` | MuJoCo viewer |
| `show_contacts` | `false` | 接触表示 |
| `publish_truth_cloud` | `true` | arena 真値点群。arena 未指定時は起動しない |
| `cloud_resolution` | `0.025` | 真値点群解像度 [m] |
| `topic_gui` | `true` | `rqt_robot_steering` |
| `flipper_gui` | `true` | `joint_state_publisher_gui` |
| `rviz` | `true` | RViz2 |

## Topic

購読:

- `/cmd_vel`, `/target/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/crawler/flipper_commands` (`std_msgs/msg/Float64MultiArray`)
- `/target/joint_states` (`sensor_msgs/msg/JointState`)

配信:

- `/joint_states`, `/crawler/joint_states` (`sensor_msgs/msg/JointState`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/imu/data`, `/crawler/imu` (`sensor_msgs/msg/Imu`)
- `/clock` (`rosgraph_msgs/msg/Clock`)
- `/octomap_pointcloud`, `/octomap_pointcloud/filtering` (arena 指定時)
- `/camera/front/*`, `/camera/rear/*`, `/camera/points` (`sensors.rgbd.enabled: true` 時)

フリッパ配列の順序は左前、左後、右前、右後で、値は rad です。

```bash
ros2 topic pub --once /crawler/flipper_commands std_msgs/msg/Float64MultiArray \
  '{data: [0.4, -0.4, 0.2, -0.2]}'
```

TF は `map -> base_link` をシミュレータが、`base_link` 以下を
`robot_state_publisher` が配信します。IMU は理想センサで covariance は 0 です。

## RGB-D

カラーは `rgb8`、depth は m 単位の `32FC1`、表示用 depth は `mono8` です。
各カメラ点群と前後結合点群は `base_link` 座標です。解像度、配信レート、取付位置、
点群間引きは `config/default.yaml` の `sensors.rgbd` で設定します。
