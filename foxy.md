# Foxy on Jetson — Setup, Build, and Test Guide

This is the practical guide for bringing `arms_ros2_control` up on a Jetson
running Ubuntu 20.04 / ROS 2 Foxy, driving a real ARX X5 arm. It picks up where
`plan.md` (the engineering record from the x86 Docker port) leaves off. Read
`plan.md` §4 first if something here doesn't work as described — it has the
reasoning behind each fix, this file just has the steps.

## 0. Before you start

- Target: Jetson, Ubuntu 20.04, ROS 2 Foxy, aarch64, real ARX X5 arm on CAN.
- Everything on `feat/foxy` branches of `ocs2_ros2` and `arms_ros2_control` was
  built and tested on an **x86_64** Docker box with `hardware:=mock_components`
  only — the real-hardware path (`hardware:=real`) could not be tested there
  because the vendored ARX SDK binaries need a newer glibc than that box had.
  The Jetson run is the first time the real-hardware path gets exercised at all.
- Give yourself a full session for the first build — see §5 for why.

## 1. System setup

```bash
# ROS 2 Foxy base, if not already installed
sudo apt update
sudo apt install -y ros-foxy-ros-base

# ros2_control stack (not in ros-base)
sudo apt install -y \
  ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-realtime-tools \
  ros-foxy-control-msgs ros-foxy-controller-manager-msgs

# OCS2 + robot description + arm driver dependencies
sudo apt install -y \
  ros-foxy-pinocchio ros-foxy-kdl-parser ros-foxy-orocos-kdl ros-foxy-spdlog-vendor \
  ros-foxy-interactive-markers ros-foxy-xacro ros-foxy-robot-state-publisher \
  ros-foxy-rviz2 ros-foxy-joint-state-publisher-gui \
  libeigen3-dev libboost-log-dev libboost-filesystem-dev liburdfdom-dev \
  libspdlog-dev libassimp-dev qtbase5-dev \
  pybind11-dev python3-pybind11 python3-numpy python3-rospkg python3-catkin-pkg \
  python3-colcon-common-extensions xterm can-utils
```

### 1.1 The one Jetson-specific step: newer libstdc++

The ARX SDK ships prebuilt `libhardware.so` / `libsolver.so` (closed source, no
source to rebuild from). Check what they actually need on your board — don't
skip this, the exact versions can vary by SDK release:

```bash
objdump -T external/arx5-sdk/lib/aarch64/*.so 2>/dev/null | \
  grep -oE 'GLIBC(XX)?_[0-9.]+' | sort -uV | tail -5
strings /usr/lib/aarch64-linux-gnu/libstdc++.so.6 | grep -oE 'GLIBCXX_[0-9.]+' | sort -uV | tail -3
```

(Run the first command after cloning in §2; the path is under
`arms_ros2_control/hardwares/arx_ros2_control/`.) On the x86 dev box, the
aarch64 `.so`s needed `GLIBC_2.17` and `GLIBCXX_3.4.29` — Focal's stock glibc
already covers `2.17`, but its libstdc++ tops out around `GLIBCXX_3.4.28`. If
your board is the same, install a newer libstdc++ runtime without touching the
system compiler:

```bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install -y libstdc++6
strings /usr/lib/aarch64-linux-gnu/libstdc++.so.6 | grep -oE 'GLIBCXX_[0-9.]+' | sort -uV | tail -3   # confirm it now covers what the SDK needs
```

Do **not** switch the default `gcc`/`g++` to a newer version for this — only the
runtime library needs bumping. If the SDK needs something this PPA doesn't
cover, you'll see it immediately as a `dlopen` failure when loading the
`arx_ros2_control` plugin (§6) — come back to this step.

## 2. Clone

The repos' submodules use `git@github.com:` URLs. If you don't have an SSH key
set up on the Jetson, rewrite each to HTTPS before `submodule update`:

```bash
mkdir -p ~/WBC && cd ~/WBC
git clone -b feat/foxy <ocs2_ros2-url> ocs2_ros2
git clone -b feat/foxy <arms_ros2_control-url> arms_ros2_control
git clone -b main <robot_descriptions-url> robot_descriptions

cd ocs2_ros2
git config submodule.submodules/ocs2_robotic_assets.url \
  https://github.com/legubiao/ocs2_robotic_assets.git
git submodule update --init submodules/ocs2_robotic_assets

cd ../arms_ros2_control
git config submodule.hardwares/arx_ros2_control.url \
  https://github.com/fiveages-sim/arx-ros2-control.git
git submodule update --init hardwares/arx_ros2_control

cd ../robot_descriptions
git config submodule.common.url \
  https://github.com/fiveages-sim/robot-descriptions-common.git
git config submodule.manipulator/ARX.url \
  https://github.com/fiveages-sim/robot-descriptions-arx.git
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

colcon build --symlink-install \
  --packages-up-to ocs2_mobile_manipulator_ros ocs2_arm_controller adaptive_gripper_controller \
                   arms_target_manager arx_ros2_control arx5_description robot_common_launch \
                   arms_rviz_control_plugin \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF
```

**Watch your RAM.** On the x86 dev box (28 cores, 62 GB) a clean build of this
package set took 2m07s. A Jetson has far fewer cores and, on the smaller
boards, 4–8 GB of RAM shared with the GPU. `colcon` parallelizes packages
across all cores by default; `ocs2_core`, `arms_target_manager`, and the
ARX SDK compile inside `arx_ros2_control` are the heaviest individual
compilation units and can OOM a Nano/Orin Nano if several build at once. If the
build gets killed (check `dmesg | grep -i oom` if so), retry with:

```bash
colcon build --symlink-install --parallel-workers 2 --executor sequential \
  --packages-up-to ... # same package list as above
```

and consider adding swap if you don't already have some (`free -h`).

Budget 15–40 minutes for the first clean build depending on the board. `-j`
inside each package's own compile (GCC parallelism per translation unit) is
unaffected by `--parallel-workers`, only how many *packages* build at once.

## 4. First launch — simulated hardware

Confirm the port itself works on aarch64 before touching CAN:

```bash
source ~/WBC/ros2_ws/install/setup.bash
export OCS2_TERMINAL_PREFIX=""   # or leave unset if you have a real terminal / X session
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=mock_components
```

Headless (no monitor / no X)? Add `rviz:=false`. If you do have a display or
are forwarding X11, RViz should show three custom panels — **OCS2 FSM**,
**Gripper Control**, **Joint Control** — alongside the model. Those panels
were ported specifically for this branch; if RViz logs `PluginlibFactory`
errors mentioning `arms_rviz_control_plugin`, something about the Qt5 install
differs from the dev box — check `qtbase5-dev` is actually installed and that
`cmake --version` isn't older than 3.10.

Expected: `joint_state_broadcaster`, `hand_controller`, `ocs2_arm_controller`
all reach `active` within a few seconds — check with:

```bash
ros2 control list_controllers
```

If `ocs2_arm_controller` fails to spawn with `Controller manager not
available`, the spawner gave up before the controller finished loading (Foxy's
`spawner.py` default timeout is 10s); this port already sets
`--controller-manager-timeout 120` on every spawner call, so if you still hit
this the controller_manager itself is stuck — check the `ros2_control_node`
log for the real error above that line.

**Control it** (no RViz needed — everything here is also a plain topic, and the
RViz panels ultimately publish the same topics):

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"   # HOLD -> OCS2  (1=HOME, 4=MOVEJ)
ros2 topic pub --once /left_target geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.08, z: 0.30}, orientation: {w: 1.0}}"   # EE target, base_link frame
ros2 topic pub --once /hand_controller/target_command std_msgs/msg/Int32 "{data: 1}"  # gripper: 0=close 1=open
```

Watch `ros2 topic echo /joint_states` (or the RViz model) — joints should move
toward the target within a couple of seconds.

**Driving the arm from the RViz marker.** Dragging the end-effector marker only
moves the marker — it does *not* send a target by itself. With the FSM in
`OCS2` (press the button in the OCS2 FSM panel, or `/fsm_command = 3`):

- one-shot: drag, release, **right-click the marker → `发送目标`**;
- continuous: **right-click → `切换到连续发布`**, then drag and the arm follows
  live (right-click → `切换到单次发布` to stop streaming).

In any other FSM state the marker is inert, and if you drag in single-shot mode
without sending, it snaps back to the controller's current target after ~1 s.
The FSM panel, the marker, and the CLI all use the same topics, so mixing them
is fine.

To drive the arm from your own planner node instead of by hand — streaming EE
poses, a `nav_msgs/Path`, or a `trajectory_msgs/JointTrajectory` — see
`plan.md` §9: it lists every input topic/service/action with its exact
semantics, which FSM state each one requires, and the timing caveats
(`time_from_start` and `Path` stamps are currently ignored). Every one of
those interfaces was exercised on the dev box with mock hardware (`plan.md` §6,
T10) — HOME, MOVEJ joint trajectory, `/left_target`, `/target_path`,
`/execute_path`, gripper open/close, and repeated `OCS2 ↔ HOLD` switching.

If anything here fails, stop and fix it before touching real hardware — every
failure mode possible with `mock_components` will only be harder to diagnose
with a live arm attached.

## 5. Bring up the real arm

1. **CAN interface.** `robot.xacro` defaults to `can1`; check what your ARX
   USB-CAN adapter actually enumerates as and pass the right one if it differs
   (see `arx5_description`'s `<hardware><param name="can_interface">`).
   Bring the link up and confirm traffic:
   ```bash
   ip link show
   sudo ip link set can1 up type can bitrate 1000000   # confirm bitrate against ARX docs
   candump can1   # Ctrl+C once you see frames; confirms wiring + termination before ROS touches it
   ```
2. **Gains.** Foxy's `ros2_control` hardware components have no ROS node, so
   there is no live `ros2 param set joint_k_gains ...` on this branch — the
   MIT-mode `kp`/`kd` gains come only from the URDF's `<hardware><param>`
   block in `arx5_description/xacro/ros2_control/robot.xacro` (and its
   sibling `interfaces.xacro`). Sane defaults are already in the file; do not
   change them casually with a live arm nearby.
3. **`mpc_frequency` is not set** in
   `arx5_description/config/ros2_control/ros2_controllers.yaml`, so the
   controller falls back to `update_rate / 4 = 12.5 Hz` for the MPC loop. Set
   `mpc_frequency` explicitly under `ocs2_arm_controller.ros__parameters` in
   that file (50 Hz is a reasonable starting point, matching `update_rate`)
   before real-hardware testing — 12.5 Hz MPC replanning against a live arm is
   sluggish.
4. **Clear stale caches** if you changed anything above:
   ```bash
   rm -rf /tmp/ocs2_ros2 ~/.ros/ocs2_cache
   ```
5. With the arm powered, CAN up, and someone's hand near the e-stop:
   ```bash
   ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real rviz:=false
   ```
   Watch the `ros2_control_node` log for `ArxX5Hardware` messages. `configure()`
   opens the CAN connection and reads URDF params; `start()` homes the arm and
   reads an initial joint state (retries up to 10× over ~1s if the first reads
   come back NaN — normal on cold CAN). If `start()` fails, the log names which
   step failed.
6. Repeat the `/fsm_command` / `/left_target` sequence from §4, arm in a clear
   space, ready to `Ctrl+C` — `stop()` runs a safe-exit into damping
   (`set_to_damping()`), optionally interpolating to a configured
   `shutdown_home` first if `shutdown_return_home` is set in the URDF params.
   Confirm both behaviors (immediate `Ctrl+C`, and after enabling
   `shutdown_return_home`) before relying on either for a real stop.

## 6. If something breaks

Foxy differs from the ROS distro this code was originally written against
(Jazzy) in ways that are easy to misdiagnose as "the arm is broken" when it's
actually an API mismatch already fixed elsewhere on this branch. Quick
symptom → cause table:

| Symptom | Likely cause |
|---|---|
| `undefined reference` at link time for a controller/HI you edited | `ament_target_dependencies` missing — Foxy's ros2_control exports no CMake targets, `${x_TARGETS}` silently expands empty |
| Controller plugin builds but pluginlib can't find the class | check `plugin_description.xml` / `*.xml` lists exactly the classes that exist — a stale entry (e.g. a class that was renamed) fails silently at `ros2 control list_controllers` load time, not at build time |
| `dlopen` failure loading `libarx_ros2_control.so` mentioning `GLIBC_2.xx` / `GLIBCXX_3.4.xx` | §1.1 — the vendored SDK `.so` needs a newer libc than installed |
| Arm HI fails at `start()` with NaN joint reads that never clear | CAN not actually up / wrong interface name / termination resistor — verify with `candump` directly, outside ROS, first |
| `ocs2_core`'s **test** targets fail to link with an undefined `dlclose` | only happens with `-DBUILD_TESTING=ON`; the fix (`${CMAKE_DL_LIBS}`) is already in `ocs2_core/CMakeLists.txt` on this branch — just don't turn tests on unless you need them |
| RViz `Could not load resource ...glb` | a stale `/tmp/ocs2_ros2/.../planning_urdf/<hash>.urdf` cache pointing at meshes before they were converted to `.dae` — `rm -rf /tmp/ocs2_ros2` |
| RViz `PluginlibFactory` error for `arms_rviz_control_plugin/*` | Qt version mismatch or the plugin didn't build — confirm `qtbase5-dev` is Qt5 (Foxy has no Qt6) and that `arms_rviz_control_plugin` is in your `--packages-up-to` list |
| `spawner.py: Controller manager not available` | spawner timed out before the controller finished configuring — should already be handled by the `--controller-manager-timeout 120` baked into the launch helpers on this branch; if you still see it, something in `ros2_control_node`'s own log failed before that |

For anything not on this list, `plan.md` §4 has the full reasoning behind every
Foxy-vs-Jazzy difference found so far, including ones that didn't end up
mattering for the ARX arm specifically (e.g. `rclcpp::Client::FutureAndRequestId`
in `arms_target_manager`'s VR path) but might matter if you touch that code.

## 7. Once it's working

Update `plan.md`'s test table (§6) and profiling section (§9) with the real
Jetson numbers — build time, cold-start time, `rt_timing_enabled` loop stats,
and process RSS all matter differently on a Jetson than they did on the x86 dev
box, and future work on this branch will want the real baseline.
