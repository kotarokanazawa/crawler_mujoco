# Bundled arena assets

`arenas/` と `models/` は、このシミュレータを単体で配布できるよう
`CrawlerRobotSimulation/Gazebo/crawler_gazebo` から必要部分のみ取り込んだものです。

- arena YAML: crawler_gazebo の `config/gazebo_environment`
- model SDF/STL/OBJ: 上記 YAML が参照する13モデルと凹形状の凸分解結果のみ
- upstream package metadata declares the source package as BSD licensed

画像、DAE、Gazebo plugin、未参照モデル、`model.config` は実行に不要なため含めていません。
`collision_parts/` の OBJ は CoACD 1.0.11 で生成済みのため、実行時に CoACD や
trimesh は必要ありません。
