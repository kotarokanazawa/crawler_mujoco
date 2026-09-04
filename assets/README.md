# Bundled arena assets

`arenas/` と `models/` は、`crawler_mujoco` を単体で配布・実行できるように
必要な資産だけをまとめたものです。

- arena YAML: `crawler_mujoco` で利用する地形設定
- model SDF/STL/OBJ: 上記 YAML が参照する13モデルと凹形状の凸分解結果のみ
- 元資産の package metadata は BSD ライセンスを宣言

画像、DAE、Gazebo plugin、未参照モデル、`model.config` は実行に不要なため含めていません。
`collision_parts/` の OBJ は CoACD 1.0.11 で生成済みのため、実行時に CoACD や
trimesh は必要ありません。
