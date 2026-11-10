# livox_mid360_measured

This is a generated runtime copy bundled with `crawler_mujoco`. The source LVX2
and extraction tools are intentionally not required by this package.

- Source: `/home/kotaro/ws_crawler_traversability/src/lidar_pattern_tools/patterns/livox_mid360_pattern/Indoor_sampledata.lvx2`
- Window: 0.500–1.500 s
- Slots: 200,000
- Measured: 129,954 (65.0%)
- Interpolated: 70,046
- Longest missing run: 384 slots
- Elevation: -7.086° to 53.912°
- Median azimuth step per four points: 1.3147°
- Median elevation steps by index mod 4: [-0.017526888713356115, -0.02160206635146967, -0.02435391043555605, 0.03469161417468619]°

Rows marked `measured=0` had no return in LVX2 and their angles were interpolated in acquisition order. The table is an empirical replay pattern, not an identification of the internal mirror mechanism.
