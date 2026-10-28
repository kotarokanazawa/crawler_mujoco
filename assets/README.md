# Bundled arena assets

`arenas/` と `models/` は、`crawler_mujoco` を単体で配布・実行できるように
必要な資産だけをまとめたものです。

- arena YAML: `crawler_mujoco` で利用する地形設定
- model SDF/STL/OBJ: 上記 YAML が参照する既存13モデル、4種類の1.2 m
  オーバーハング障害物、および凹形状の凸分解結果
- 元資産の package metadata は BSD ライセンスを宣言

画像、DAE、Gazebo plugin、未参照モデル、`model.config` は実行に不要なため含めていません。
`collision_parts/` の OBJ は CoACD 1.0.11 で生成済みのため、実行時に CoACD や
trimesh は必要ありません。

`arenas/overhang_environment.yaml` は `DESK`、`KO`、`NEGO`、`TONNEL` を
同時に配置するオーバーハング評価用環境です。STLの座標単位はmmなので、各
`model.sdf` でX軸回りに+90度補正し、`0.01`倍して基準辺を1.2 mにしています。
各モデルの+X側には、世界座標Z方向だけを0.5倍した高さ0.6 m版も配置します。
