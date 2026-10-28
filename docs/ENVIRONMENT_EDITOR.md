# MuJoCo環境エディタ

`crawler_mujoco`だけで動作するブラウザベースのarenaエディタです。
編集内容は上面・側面ビューに加え、実際のMuJoCo offscreen rendererによる
透視3D画像へ自動反映されます。

## 起動

```bash
ros2 launch crawler_mujoco environment_editor.launch.py
```

通常はブラウザが自動的に開きます。手動で開く場合は次を実行し、
`http://127.0.0.1:8766`へアクセスします。

```bash
ros2 launch crawler_mujoco environment_editor.launch.py open_browser:=false
```

## 操作

- 上部の一覧でパッケージ内のarena YAMLを選び、「一覧から読込」で開く
- 任意の既存`.yaml` / `.yml`は「YAMLを開く」から選択して開く
- モデル選択欄からモデルを選び、「選択モデルを追加」でspawn
- 上面ビューを左ドラッグしてX/Y移動
- 側面ビューを左ドラッグしてX/Z移動
- 上面ビューを右ドラッグしてYaw回転
- 側面ビューを右ドラッグしてPitch回転
- Shiftを押しながら移動すると0.1 m刻み
- Shiftを押しながら回転すると15度刻み
- マウスホイールで表示倍率変更
- 移動、回転、スケール変更後にMuJoCo 3Dプレビューを自動更新
- 右側の入力欄で位置、RPY、モデルローカルXYZスケールを直接指定
- 選択オブジェクトの複製と削除
- 「戻す」「進む」、または`Ctrl+Z` / `Ctrl+Y`（`Ctrl+Shift+Z`も可）で編集履歴を移動
- 3Dプレビューをドラッグして視点を回転、ホイールでズーム
- 3Dプレビュー右上のスライダーで、3D領域と上面・側面領域の高さを変更
- 右側の「arena YAML」で現在の内容をYAML形式のまま確認・編集
- YAMLを直接変更した場合は「YAMLテキストを画面へ反映」で3D・上面・側面へ反映

## 保存と実行

一覧から読み込んだYAMLは「読込元へ上書き」で元ファイルへ保存できます。
ファイル名欄へ新しい名前を入力して「別名で保存」を押した場合、保存先は
`assets/arenas/custom/`です（`.yaml`は自動的に付きます）。元のYAMLは上書き
されません。保存後、画面上部に絶対パスが表示され、保存したYAMLが一覧で
選択されます。

「YAMLを開く」でブラウザから選択したファイルは、ブラウザのセキュリティ制約に
より元ファイルへ直接書き戻せません。最初に「別名で保存」すると、その保存先を
以後「読込元へ上書き」で更新できます。

```bash
ros2 launch crawler_mujoco crawler_mujoco.launch.py \
  arena_yaml:=/absolute/path/to/assets/arenas/custom/my_environment.yaml
```

オーバーハング用のセンサ・点群設定で実行する場合は、`custom/`からの相対パスを
指定できます。

```bash
ros2 launch crawler_mujoco overhang_environment.launch.py \
  arena_yaml:=custom/my_environment.yaml
```

ロボットを配置せず、環境の3D voxel点群だけを確認する例:

```bash
ros2 launch crawler_mujoco crawler_mujoco.launch.py \
  arena_yaml:=/absolute/path/to/my_environment.yaml \
  spawn_robot:=false \
  pointcloud_mode:=3d_voxel
```
