# Foxy on Jetson — Setup, Build, and Test Guide

This is the practical guide for bringing `arms_ros2_control` up on a Jetson
running Ubuntu 20.04 / ROS 2 Foxy, driving a real ARX X5 arm. It picks up where
`plan.md` (the engineering record from the x86 Docker port) leaves off. Read
`plan.md` §4 first if something here doesn't work as described — it has the
reasoning behind each fix, this file just has the steps.

## 0. Before you start

- Target: Jetson, Ubuntu 20.04, ROS 2 Foxy, aarch64, real ARX X5 arm on CAN.

## 1. System setup

```bash
# ROS 2 Foxy base, if not already installed
sudo apt update
sudo apt install -y ros-foxy-ros-base

# ros2_control stack (not in ros-base)
sudo apt install -y ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-realtime-tools ros-foxy-control-msgs ros-foxy-controller-manager-msgs

# OCS2 + robot description + arm driver dependencies
sudo apt install -y ros-foxy-pinocchio ros-foxy-kdl-parser ros-foxy-orocos-kdl ros-foxy-spdlog-vendor ros-foxy-interactive-markers ros-foxy-xacro ros-foxy-robot-state-publisher ros-foxy-rviz2 ros-foxy-joint-state-publisher-gui libeigen3-dev libboost-log-dev libboost-filesystem-dev liburdfdom-dev libspdlog-dev libassimp-dev qtbase5-dev pybind11-dev python3-pybind11 python3-numpy python3-rospkg python3-catkin-pkg python3-colcon-common-extensions xterm can-utils
```

## 2. Clone

The repos' submodules use `git@github.com:` URLs. If you don't have an SSH key
set up on the Jetson, rewrite each to HTTPS before `submodule update`:

```bash
mkdir -p ~/WBC && cd ~/WBC
git clone -b feat/foxy <ocs2_ros2-url> ocs2_ros2
git clone -b feat/foxy <arms_ros2_control-url> arms_ros2_control
git clone -b feat/foxy <robot_descriptions-url> robot_descriptions

cd ocs2_ros2
git submodule update --init submodules/ocs2_robotic_assets

cd ../arms_ros2_control
git submodule update --init hardwares/arx_ros2_control

cd ../robot_descriptions
git submodule update --init common "manipulator/ARX"
```

Set up the workspace:

```bash
mkdir -p ~/WBC/ros2_ws/src
cd ~/WBC/ros2_ws/src
ln -s ../../ocs2_ros2 .
ln -s ../../arms_ros2_control .
ln -s ../../robot_descriptions .
```

## 3. Build

```bash
cd ~/WBC/ros2_ws
source /opt/ros/foxy/setup.bash

colcon build --symlink-install --packages-up-to ocs2_mobile_manipulator_ros ocs2_arm_controller adaptive_gripper_controller arms_target_manager arx_ros2_control arx5_description robot_common_launch arms_rviz_control_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF
```

```bash
colcon build --symlink-install --parallel-workers 2 --executor sequential --packages-up-to ... # same package list as above
```

## 4. Simulation

Confirm the port itself works on aarch64 before touching CAN:

```bash
source ~/WBC/ros2_ws/install/setup.bash
export OCS2_TERMINAL_PREFIX=""   # or leave unset if you have a real terminal / X session
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=mock_components
```

Headless (no monitor / no X)? Add `rviz:=false`. If you do have a display or are forwarding X11, RViz should show three custom panels — **OCS2 FSM**, **Gripper Control**, **Joint Control** — alongside the model. 

**Control it** (no RViz needed — everything here is also a plain topic, and the RViz panels ultimately publish the same topics):

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"   # HOLD -> OCS2  (1=HOME, 4=MOVEJ)
ros2 topic pub --once /left_target geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.08, z: 0.30}, orientation: {w: 1.0}}"   # EE target, base_link frame
ros2 topic pub --once /hand_controller/target_command std_msgs/msg/Int32 "{data: 1}"  # gripper: 0=close 1=open
```

Watch `ros2 topic echo /joint_states` (or the RViz model) — joints should move toward the target within a couple of seconds.

## 5. Bring up the real arm

1. **CAN interface.** `robot.xacro` defaults to `can1`; check what your ARX USB-CAN adapter actually enumerates as and pass the right one if it differs (see `arx5_description`'s `<hardware><param name="can_interface">`).
   Bring the link up and confirm traffic:

   ```bash
   ip link show
   sudo ip link set can1 up type can bitrate 1000000   # confirm bitrate against ARX docs
   candump can1   # Ctrl+C once you see frames; confirms wiring + termination before ROS touches it
   ```
2. **Gains.** Foxy's `ros2_control` hardware components have no ROS node, so there is no live `ros2 param set joint_k_gains ...` on this branch — the MIT-mode `kp`/`kd` gains come only from the URDF's `<hardware><param>` block in `arx5_description/xacro/ros2_control/robot.xacro` (and its
   sibling `interfaces.xacro`). Sane defaults are already in the file; do not change them casually with a live arm nearby.
3. **`mpc_frequency` is not set** in
   `arx5_description/config/ros2_control/ros2_controllers.yaml`, so the controller falls back to `update_rate / 4 = 12.5 Hz` for the MPC loop. Set`mpc_frequency` explicitly under `ocs2_arm_controller.ros__parameters`
4. **Clear stale caches** if you changed anything above:

   ```bash
   rm -rf /tmp/ocs2_ros2 ~/.ros/ocs2_cache
   ```
5. With the arm powered, CAN up, and someone's hand near the e-stop:

   ```bash
   ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real rviz:=false
   ```

   Watch the `ros2_control_node` log for `ArxX5Hardware` messages. `configure()`opens the CAN connection and reads URDF params; `start()` homes the arm and reads an initial joint state (retries up to 10× over ~1s if the first reads come back NaN — normal on cold CAN). If `start()` fails, the log names which step failed.

## 6. If something breaks

Foxy differs from the ROS distro this code was originally written against
(Jazzy) in ways that are easy to misdiagnose as "the arm is broken" when it's
actually an API mismatch already fixed elsewhere on this branch. Quick
symptom → cause table:

| Symptom                                                                                              | Likely cause                                                                                                                                                                                                                                                            |
| ---------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `undefined reference` at link time for a controller/HI you edited                                  | `ament_target_dependencies` missing — Foxy's ros2_control exports no CMake targets, `${x_TARGETS}` silently expands empty                                                                                                                                          |
| Controller plugin builds but pluginlib can't find the class                                          | check`plugin_description.xml` / `*.xml` lists exactly the classes that exist — a stale entry (e.g. a class that was renamed) fails silently at `ros2 control list_controllers` load time, not at build time                                                      |
| `dlopen` failure loading `libarx_ros2_control.so` mentioning `GLIBC_2.xx` / `GLIBCXX_3.4.xx` | §1.1 — the vendored SDK`.so` needs a newer libc than installed                                                                                                                                                                                                      |
| Arm HI fails at`start()` with NaN joint reads that never clear                                     | CAN not actually up / wrong interface name / termination resistor — verify with`candump` directly, outside ROS, first                                                                                                                                                |
| `ocs2_core`'s **test** targets fail to link with an undefined `dlclose`                    | only happens with`-DBUILD_TESTING=ON`; the fix (`${CMAKE_DL_LIBS}`) is already in `ocs2_core/CMakeLists.txt` on this branch — just don't turn tests on unless you need them                                                                                      |
| RViz`Could not load resource ...glb`                                                               | a stale`/tmp/ocs2_ros2/.../planning_urdf/<hash>.urdf` cache pointing at meshes before they were converted to `.dae` — `rm -rf /tmp/ocs2_ros2`                                                                                                                    |
| RViz`PluginlibFactory` error for `arms_rviz_control_plugin/*`                                    | Qt version mismatch or the plugin didn't build — confirm`qtbase5-dev` is Qt5 (Foxy has no Qt6) and that `arms_rviz_control_plugin` is in your `--packages-up-to` list                                                                                            |
| `spawner.py: Controller manager not available`                                                     | spawner timed out before the controller finished configuring — should already be handled by the`--controller-manager-timeout 120` baked into the launch helpers on this branch; if you still see it, something in `ros2_control_node`'s own log failed before that |
