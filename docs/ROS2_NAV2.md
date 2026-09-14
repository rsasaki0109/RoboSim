# Running Nav2 Against RNE

`nav_node.py` is a Nav2-facing RNE bridge. It subscribes `/cmd_vel`, integrates
a planar differential-drive pose, and publishes the topics a Nav2 stack needs:
`/clock`, `/odom`, `/tf`, `/scan`, `/map`, and `/plan`. Unlike `run_node.py` it
does not depend on `simulation_interfaces`.

This integration is verified end to end: `nav2_integration.sh` starts the bridge,
launches `nav2_bringup navigation_launch.py`, sends a `NavigateToPose` goal, and
confirms Nav2's controller drives the RNE base to the goal
(`Reached the goal!` / `Goal succeeded`).

## Prerequisites

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-simulation-interfaces ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
```

The default `nav2_bringup` parameters target the TurtleBot3 frame layout
`map → odom → base_footprint → base_link → lidar`, so the bridge publishes all
of those frames.

## Start the RNE bridge

```bash
source /opt/ros/jazzy/setup.bash
cd adapters/ros2/rne_ros2_bridge
python3 nav_node.py
```

Verify:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
ros2 topic echo /odom --field pose.pose.position.x --once
```

The included `nav_smoke.sh` runs this check automatically.

## Full Nav2 bring-up

```bash
bash adapters/ros2/rne_ros2_bridge/nav2_integration.sh
```

or manually:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0}, orientation: {w: 1.0}}}}"
```

## Frames and QoS

- `/tf` publishes `map → odom → base_footprint → base_link → lidar` with yaw
  encoded on `odom → base_footprint`.
- `/map` is published with a latched (`TRANSIENT_LOCAL`) QoS so Nav2's static
  costmap layer receives it.
- Because RNE drives `/clock`, set `use_sim_time:=true` for every Nav2 node.

## Real SLAM map

By default `nav_node.py` publishes a synthetic walled room. To plan on a map
built from real simulated LiDAR, generate one and point the bridge at it:

```bash
RNE_SLAM_MAP_DIR=/tmp/rne-map cargo run -p nav_slam_physics --example 98_nav_slam_physics
RNE_MAP_FILE=/tmp/rne-map/map.yaml python3 nav_node.py
```

`nav_node.py` reads the `map_server` PGM+YAML pair and republishes it on
`/map`; `slam_map_smoke.sh` verifies this. `/joint_states` publishes the wheel
joint positions/velocities for the ros2_control boundary.

## Connect Nav2 with custom parameters

If your robot uses `base_link` as the base frame instead of `base_footprint`,
override `robot_base_frame` in the controller, planner, and costmap sections of
your params file.

## Determinism

The bridge only transports typed messages; all simulation and SLAM logic stays
in the ROS-free `rne_nav` / `rne_slam` crates, so a Nav2 run is reproducible
when `/cmd_vel` is replayed.

## Limits

- The synthetic scan is a walled room; replace `_ray_to_wall` with an
  `rne_sensor` scan or a `rne_slam` map for a real scene.
- `nav_node.py` does not expose `simulation_interfaces`; use `run_node.py` when
  those services are required.
