# リリース確認

## 事前確認

1. `package.xml` の version、maintainer、license を確認する。
2. `CHANGELOG.rst` に対象 version の変更を記載する。
3. `THIRD_PARTY_NOTICES.md` と同梱 arena 資産を一緒に配布する。
4. `.venv/`、`.cache/`、`results/`、`__pycache__/` が配布物にないことを確認する。
5. Python 構文、MJCF、同梱 arena、ROS 2 build を検証する。

```bash
./setup.sh
PYTHONDONTWRITEBYTECODE=1 ./.venv/bin/python -m compileall -q .
./.venv/bin/python -m unittest discover -s test -v

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_crawler
```

## 動作確認

```bash
./.venv/bin/python scripts/mujoco_crawler --shape all --output /tmp/mujoco-smoke
./.venv/bin/python scripts/mujoco_crawler \
  --shape rectangle --arena-yaml singlerane/bridge.yaml \
  --write-xml-only --output /tmp/mujoco-arena-smoke
```

GUI と ROS 2 は自動テストだけでは描画・topic 接続を保証できないため、タグ作成前に
viewer、`/cmd_vel`、フリッパ指令、RViz の点群を一度確認してください。
