# MuJoCo Crawler

フリッパ付きクローラを MuJoCo で走行させる、単体実行可能なシミュレータです。
主履帯とフリッパ履帯のグローサ形状を切り替え、接触力・姿勢・到達時間を CSV に記録できます。
ROS 2 ブリッジ、RGB-D、真値地形点群の配信にも対応します。

`crawler_gazebo` の arena YAML と必要な STL/SDF は `assets/` に同梱しているため、
別のシミュレーションリポジトリは不要です。

## 動作環境

- Linux
- Python 3.10 以上
- GUI 利用時は OpenGL が使用可能な環境
- ROS 2 連携時のみ ROS 2 Humble と PCL

## クイックスタート

内蔵 bridge を使う最小構成です。

```bash
./setup.sh
./.venv/bin/python scripts/mujoco_crawler --shape rectangle --gui
```

複数の Python がある環境では `CRAWLER_PYTHON=/path/to/python3 ./setup.sh` で
Python 3.10 以上の実行ファイルを指定できます。

ブラウザ操作画面も開く場合:

```bash
./.venv/bin/python scripts/mujoco_crawler --shape semicircle --gui --control-ui
```

ブラウザは既定で `http://127.0.0.1:8765` を使用します。

## 同梱 arena

凹地形の凸分解済み mesh も同梱しているため、追加パッケージは不要です。

```bash
./.venv/bin/python scripts/mujoco_crawler \
  --shape rectangle --arena-yaml benchmark.yaml --gui
```

相対パスは `assets/arenas/` から解決されます。例えば
`singlerane/bridge.yaml`、`singlerane/stair.yaml`、
`singlerane/stepfield.yaml` を指定できます。独自 YAML の絶対パスも受け付けますが、
参照する `model://` 資産は `assets/models/` に存在する必要があります。

## 主な実行例

```bash
# 接触点と接触力を表示
./.venv/bin/python scripts/mujoco_crawler --shape spike --gui --show-contacts

# 全形状をヘッドレス比較し、CSV とグラフを生成
./.venv/bin/python scripts/mujoco_crawler --shape all

# MJCF の生成だけを行う
./.venv/bin/python scripts/mujoco_crawler --shape semicircle --write-xml-only

# フリッパ角度追従を検証
./.venv/bin/python -m tools.validate_flipper_control
```

形状は `none`、`rectangle`、`semicircle`、`spike` の4種類です。
出力先は既定で `results/`、変更する場合は `--output PATH` を指定します。

## ROS 2

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select mujoco_crawler
source install/setup.bash
ros2 launch mujoco_crawler mujoco_ros2.launch.py shape:=semicircle
```

シミュレータだけを起動する場合は `ros2 run` も使用できます。

```bash
ros2 run mujoco_crawler mujoco_crawler --shape rectangle --ros2
```

arena を使う場合は launch に
`python_executable:=/path/to/Mujoco_crawler/.venv/bin/python` と
`arena_yaml:=benchmark.yaml` を渡します。詳しい topic、launch 引数、依存パッケージは
[docs/ROS2.md](docs/ROS2.md) を参照してください。

## 設定と出力

シミュレーション、センサ、グローサ、環境は `config/default.yaml`、
ロボット寸法と制御ゲインは `config/robot.yaml` で設定します。項目の説明は
[docs/CONFIGURATION.md](docs/CONFIGURATION.md) にまとめています。

## ディレクトリ構成

- `mujoco_crawler/`: Pythonシミュレータ、ROSブリッジ、設定・arena読込
- `src/`: ROS 2 C++点群ノード
- `scripts/`: `ros2 run` と単体実行のエントリポイント
- `launch/`, `config/`, `rviz/`: ROS 2共有資産
- `assets/`: arena YAML、SDF/STL、凸分解済みcollision mesh
- `tools/`: 手動検証ツール
- `test/`: 自動スモークテスト
- `docs/`: 設定、ROS 2、リリース手順

主な生成物:

- `crawler_<shape>.xml`: 実際に読み込んだ MJCF
- `<shape>.csv`: 位置、pitch、接触数、法線力、接線力
- `summary.csv`: 到達位置、到達時間、最大 pitch
- `shape_comparison.png`: `--shape all` の比較グラフ

## モデル上の制約

本モデルは形状差と接触応答の比較を目的とします。履帯は回転部と周期的に循環する
直動部で近似しており、グローサ同士をリンク拘束した完全な履帯ではありません。
実機や Gazebo の結果と比較する際は、このモデル差を考慮してください。

リリース確認手順は [docs/RELEASE.md](docs/RELEASE.md)、変更履歴は
[CHANGELOG.rst](CHANGELOG.rst) を参照してください。ソースコードは Apache-2.0、
同梱 arena 資産は upstream 宣言に基づく BSD です。詳細は
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。
