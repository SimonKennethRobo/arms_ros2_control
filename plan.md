# Foxy Port Plan — `arms_ros2_control` (branch `feat/foxy`)

This file is a hand-off prompt for whoever continues this work (human or model).
It records what was found, what was changed, how it was tested, and what is left.

## 0. Goal and constraints

- Run `ocs2_arm_controller` (OCS2 MPC inside a ros2_control controller) on a real
  **ARX X5** arm through `arx_ros2_control`, on a **Jetson running Ubuntu 20.04 /
  ROS 2 Foxy (aarch64)**.
- Branch `feat/foxy` is **Foxy-only**. Do not keep Jazzy compatibility here.
- Keep scope tight: only packages on the path
  `arx_ros2_control -> ocs2_arm_controller -> demo.launch.py` are ported.
  Everything else (other hardware drivers, `ocs2_wbc_controller`, `arms_teleop`,
  `arms_rviz_control_plugin`, `ocs2_humanoid`, `lina_planning`) is left untouched
  and simply not built.
- Dev box used so far: Ubuntu 20.04 **x86_64** Docker with Foxy; RViz viewed via
  NoMachine on `DISPLAY=:1001`. Real-hardware runs must happen on the Jetson (see §6).

## 1. Workspace layout

```
~/WBC/ros2_ws/src/
  ocs2_ros2           -> ~/WBC/ocs2_ros2            (branch feat/foxy)
  arms_ros2_control   -> ~/WBC/arms_ros2_control    (branch feat/foxy)
  robot_descriptions  -> ~/WBC/robot_descriptions   (branch main; submodules common, manipulator/ARX)
```

Submodules use `git@github.com:` URLs. Without SSH keys, fetch them over HTTPS:

```bash
git config submodule.<name>.url https://github.com/fiveages-sim/<repo>.git
git submodule update --init <path>
```

Initialised so far: `ocs2_ros2/submodules/ocs2_robotic_assets`,
`arms_ros2_control/hardwares/arx_ros2_control`,
`robot_descriptions/common` (advanced to upstream `origin/main` — the pinned commit
lacked `resolve_robot_arms` / `planning_robot_for_arm_family`),
`robot_descriptions/manipulator/ARX`.

## 2. System packages

Already installed on the dev box (verified). On a fresh Jetson install:

```bash
sudo apt install -y \
  ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-realtime-tools \
  ros-foxy-control-msgs ros-foxy-controller-manager-msgs \
  ros-foxy-pinocchio ros-foxy-kdl-parser ros-foxy-orocos-kdl ros-foxy-spdlog-vendor \
  ros-foxy-interactive-markers ros-foxy-xacro ros-foxy-robot-state-publisher ros-foxy-rviz2 \
  ros-foxy-joint-state-publisher-gui \
  libeigen3-dev libboost-log-dev libboost-filesystem-dev liburdfdom-dev libspdlog-dev \
  pybind11-dev python3-pybind11 python3-numpy python3-rospkg python3-catkin-pkg xterm
```

Jetson-only extra (see §6): a libstdc++ providing `GLIBCXX_3.4.29` (GCC 11 runtime).

Not needed: CppAD/CppADCodeGen (vendored in `ocs2_thirdparty`), HPIPM/BLASFEO
(built in-tree), `orocos_kdl_vendor`/`mock_components` (do not exist on Foxy).

## 3. Minimal dependency set for `ocs2_arm_controller`

Derived from actual `#include`s, verified by building. Packages built for the demo:

| Package | Repo | Notes |
|---|---|---|
| `ocs2_thirdparty, ocs2_core, ocs2_oc, ocs2_mpc, ocs2_qp_solver, ocs2_ddp, blasfeo_colcon, hpipm_colcon, ocs2_sqp, ocs2_msgs, ocs2_ros_interfaces, ocs2_robotic_tools, ocs2_pinocchio_interface, ocs2_self_collision, ocs2_self_collision_visualization, ocs2_mobile_manipulator, ocs2_robotic_assets` | ocs2_ros2 | `--packages-up-to ocs2_mobile_manipulator_ros` |
| `arms_ros2_control_msgs` | arms_ros2_control | |
| `arms_controller_common` | arms_ros2_control | FSM, kinematics, gravity comp |
| `ocs2_controller_common` | arms_ros2_control | reference manager, visualizer |
| `ocs2_arm_controller` | arms_ros2_control | the controller plugin |
| `adaptive_gripper_controller` | arms_ros2_control | `hand_controller` in the ARX yaml |
| `arms_target_manager` | arms_ros2_control | RViz interactive EE marker |
| `arx_ros2_control` | arms_ros2_control (submodule) | real hardware plugin |
| `robot_common_launch, component_models, sensor_models, arx5_description` | robot_descriptions | launch helpers + URDF/meshes |

Dependencies **removed** as unnecessary: `lina_planning`, `ocs2_wheel_humanoid`,
`ocs2_mobile_manipulator_ros` from `ocs2_controller_common`; `lina_planning` from
`arms_controller_common`; `arms_rviz_control_plugin` from `arms_target_manager`
(that package never used it).
Dependencies **added** because they were used but undeclared: `pinocchio`,
`hardware_interface`, `rclcpp(_lifecycle)` (ocs2_arm_controller); `trajectory_msgs`,
`std_msgs`, `rclcpp_action` (arms_controller_common); the six `ocs2_*` packages
`ocs2_controller_common` actually `find_package`s; `sensor_msgs` (arms_target_manager).

An earlier pass of this port mistakenly also stripped `robot_common_launch`,
`arms_target_manager`, `rviz2` from `ocs2_arm_controller`'s `exec_depend`s, reasoning
they were "launch-time only" — that is backwards; `exec_depend` is precisely for
launch-time-only dependencies, and `demo.launch.py` genuinely imports/launches all
three. The mistake did not surface in testing because colcon does not enforce
`package.xml` at runtime inside one workspace. **This has been corrected** — see §4.7.

## 4. Foxy differences found and how they were handled

### 4.1 ros2_control 0.11 controller API (`ocs2_arm_controller`, `adaptive_gripper_controller`)
- No `on_init()` → `return_type init(const std::string&)`, which must call
  `ControllerInterface::init()` first (that creates `node_`).
- `update()` has no time/period → derive them from `get_node()->now()` and the
  previous call (`Ocs2ArmController::update()`).
- No `controller_interface::CallbackReturn` → local alias to
  `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn`.
- `get_node()` returns `rclcpp::Node`, not a `LifecycleNode` → all
  `rclcpp_lifecycle::LifecycleNode` uses in `arms_controller_common`,
  `ocs2_controller_common`, `ocs2_arm_controller` replaced with `rclcpp::Node`.
- Handles: `get_optional()` → `get_value()`, `bool set_value()` → `void set_value()`.
  A shim `arms_controller_common/HardwareInterfaceCompat.h` keeps the
  `std::optional` shape at the call sites; `std::ignore =` prefixes removed.
- `LoanedCommandInterface::get_prefix_name()` → `get_name()` (returns the joint name on Foxy).
- `controller_manager` does not push `update_rate` to controllers → declared with
  `auto_declare` and a default.
- ros2_control 0.11 exports **no CMake targets**: `${controller_interface_TARGETS}` /
  `${hardware_interface_TARGETS}` are empty → `ament_target_dependencies(...)`.

### 4.2 ros2_control 0.11 hardware API (`arx_ros2_control`)
`ArxX5Hardware` was rewritten against `BaseInterface<SystemInterface>`:
`configure(info)` (parse URDF params + open CAN) → `start()` (home, seed buffers,
push MIT gains) → `stop()` (safe-exit, damping, disconnect); `read()`/`write()`
take no arguments; handles are returned by value. Hardware components have no node
on Foxy, so the dynamic `ros2 param set joint_k_gains` tuning is gone — gains come
from `<hardware><param>` in the URDF only. The Lift2S interface
(`arx_lift_hardware.cpp`) still uses the Jazzy API and is excluded by
`ARX_FOXY_ARM_ONLY ON` in `CMakeLists.txt` (its `.so` is x86_64-only anyway).
`package.xml`: `orocos_kdl_vendor` → `orocos_kdl`.

### 4.3 rclcpp / Python / tooling
- Subscription callbacks must take `Msg::ConstSharedPtr` **by value**, not
  `const ConstSharedPtr&` (38 sites in `arms_target_manager`).
- `rclcpp::Client::FutureAndRequestId` / `remove_pending_request` are Humble+ →
  `SharedFuture` in `VRInputHandler`.
- tf2 headers are `.h` on Foxy (`tf2_ros/buffer.h`, `tf2_geometry_msgs/tf2_geometry_msgs.h`, …).
- `rclcpp::Duration::from_nanoseconds` missing → explicit constructor.
- GCC 9 defaults to gnu++14 → `target_compile_features(... cxx_std_17)` and
  `-lstdc++fs` added where `std::filesystem`/`std::clamp` are used.
- Python 3.8: PEP 585 generics in annotations need `from __future__ import annotations`
  (`robot_common_launch`).
- Foxy `ros2 topic echo` has no `--once`; use a small rclpy script instead.
- Foxy's `RCLCPP_*_THROTTLE` macros capture the clock argument **by reference in a
  lambda** that runs later; passing a temporary (`*std::make_shared<rclcpp::Clock>()`,
  `StateHome.cpp:252`) dangles and surfaces at runtime as
  `RCLCPP_WARN_THROTTLE could not get current time stamp` + `getting current steady
  time failed` (found in T10 on entering `HOME`; harmless to motion, but UB). Fixed
  by passing `*node_->get_clock()`. It was the only such site in the tree (grepped
  all 44 throttle calls).

### 4.4 Launch / controller_manager (`robot_common_launch`, `_ocs2_launch_common.py`)
- Spawner executable is `spawner.py` → `spawner_executable()` probe helper.
- `ros2_control_node` needs `robot_description` as a **parameter** (topic-based
  loading is Humble+).
- `spawner.py -p file.yaml` wants the controller keyed as `/<name>`; the param
  file now carries both `<name>` and `/<name>` keys.
- `spawner.py` gives up after 10 s by default; OCS2 controller loading takes longer
  → `--controller-manager-timeout 120` on every spawner.
- Fake hardware plugin is `fake_components/GenericSystem` (renamed `mock_components`
  in Iron) → `arx5_description/xacro/ros2_control/robot.xacro` emits the Foxy name
  while keeping `ros2_control_hardware_type:=mock_components` as the argument.

### 4.5 RViz
- Foxy RViz (Ogre 1.12) cannot load `.glb`. All ARX / sensor / component meshes were
  converted to `.dae` with a 30-line assimp tool (`libassimp-dev` 5.0.1 is on Focal) and
  every `.glb` reference in `arx5_description`, `sensor_models`, `component_models`
  URDF/xacro was rewritten. The `.glb` originals are still in the tree.
- `rviz_common/Time` and the three `arms_rviz_control_plugin` panels were removed from
  `ocs2_arm_controller/config/demo.rviz`; FSM/gripper commands go over topics (§5).
- **Stale planning-URDF cache**: `robot_common_launch` caches the OCS2 planning URDF
  under `/tmp/ocs2_ros2/<robot>/planning_urdf/<hash>.urdf`, and the hash does not
  change when mesh paths change. After converting meshes, delete that directory or
  the second `RobotModel` display keeps asking for `.glb`.

### 4.6 OCS2 core (`ocs2_ros2`, already fixed on its `feat/foxy`)
- glibc 2.31 keeps `dlclose` in a separate `libdl` → `ocs2_core` links `${CMAKE_DL_LIBS}`.
  Only visible with `BUILD_TESTING=ON`; build with `-DBUILD_TESTING=OFF`.
- `ocs2_robotic_assets` is a submodule and must be initialised or the demos fail at launch.

### 4.7 `arms_rviz_control_plugin` — ported and kept (not dropped)
This package (`OCS2FSMPanel`, `GripperControlPanel`, `JointControlPanel`,
`WbcCapabilityPanel`; ~4900 lines total) turned out to have **no Jazzy-only rclcpp
API** in it at all — none of the patterns in §4.3 appear anywhere in its source. The
only real blocker was its `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.18)` +
`cmake_language(CALL qt${QT_VERSION_MAJOR}_wrap_cpp ...)`, used upstream to pick Qt5
vs. Qt6 at configure time. Focal's apt `cmake` is 3.16.3 (no `cmake_language(CALL)`),
and Focal never ships Qt6 in the first place (rviz2 there is Qt5-only) — so the
Foxy port just hardcodes Qt5 (`find_package(Qt5 REQUIRED COMPONENTS Widgets)` +
`qt5_wrap_cpp`) and drops `cmake_minimum_required` back to 3.10. `getDisplayContext()
->getRosNodeAbstraction().lock()->get_raw_node()`, the pattern every panel uses to
get a node handle, exists unchanged on Foxy's `rviz_common`.

`ocs2_arm_controller/config/demo.rviz` keeps `OCS2FSMPanel` / `GripperControlPanel`
/ `JointControlPanel` (not `WbcCapabilityPanel` — it was never in the original
config). Only `rviz_common/Time` was removed from the panel list: that is a
built-in rviz_common panel (not part of this plugin) that does not exist before
Humble — Foxy's `PluginlibFactory` reports no such class at all.

`ocs2_arm_controller/package.xml` now `exec_depend`s on `arms_rviz_control_plugin`.
Verified with the panels actually loaded (not just building): `libarms_rviz_control_plugin.so`
appears in `rviz2`'s `/proc/<pid>/maps`, zero `PluginlibFactory` errors in the log,
and `ros2 node info /rviz` shows publishers on `/fsm_command`,
`/hand_controller/target_command`, `/hand_controller/target_percent` — i.e. the FSM
and gripper panels constructed successfully and are live.

## 5. How to build and run (dev box or Jetson)

```bash
cd ~/WBC/ros2_ws && source /opt/ros/foxy/setup.bash
colcon build --symlink-install \
  --packages-up-to ocs2_mobile_manipulator_ros ocs2_arm_controller adaptive_gripper_controller \
                   arms_target_manager arx_ros2_control arx5_description robot_common_launch \
                   arms_rviz_control_plugin \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF

rm -rf /tmp/ocs2_ros2 ~/.ros/ocs2_cache   # drop stale caches after URDF/mesh/task.info changes
source install/setup.bash
export DISPLAY=:1001 OCS2_TERMINAL_PREFIX=""
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=mock_components   # sim
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real              # Jetson + arm
```

Controlling the arm (no RViz panels on Foxy):

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"      # HOLD -> OCS2 (1 = HOME, 4 = MOVEJ)
ros2 topic pub --once /left_target geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.08, z: 0.30}, orientation: {w: 1.0}}"      # EE target in base_link
# or drag the interactive marker published by arms_target_manager in RViz
ros2 topic pub --once /hand_controller/target_command std_msgs/msg/Int32 "{data: 1}"  # gripper 0=close 1=open
```

`ocs2_arm_controller` caches its CppAD-generated dynamics libraries under
`~/.ros/ocs2_cache/<robot_pkg>/` (falls back to `$XDG_CACHE_HOME/ocs2_ros2/<robot_pkg>`,
then `/tmp/ocs2_ros2/<robot_pkg>` if `$HOME` is unset) — **not** the
`/tmp/ocs2_ros2/arx5/planning_urdf/` path, which is a separate, smaller xacro-URDF
cache written by `robot_common_launch` (see §4.5). See §10 for how long the cold
compile actually takes.

## 6. Test plan and status

| # | Test | Status on x86 Docker | Jetson |
|---|---|---|---|
| T1 | `colcon build` of the full package set in §5, from clean (`rm -rf build install log`) | **PASS** — 29 packages, 0 errors, 2m07s wall (28 cores) | todo |
| T2 | `demo.launch.py robot:=arx5 hardware:=mock_components` brings `joint_state_broadcaster`, `hand_controller`, `ocs2_arm_controller` to `active` with no manual spawn | **PASS** (repeated across 4 independent launches) | todo |
| T3 | `/fsm_command=3` then `/left_target` pose → arm joints move | **PASS** — max Δ ≈ 0.54–0.80 rad depending on start pose, repeatable | todo |
| T4 | `arms_target_manager` publishes an interactive marker (`/arms_target_manager/update`, 1 marker) | **PASS** | todo |
| T5 | RViz renders the robot mesh (`.dae`), no `PluginlibFactory` / `Could not load resource` errors | **PASS** after clearing `/tmp/ocs2_ros2` (0 errors in log) | todo |
| T6 | `libarx_ros2_control.so` exports Foxy symbols (`configure/start/stop/read/write`) and links | **PASS** (link only — `nm -DC` confirms Foxy symbols, no Jazzy `on_init`/`on_export_*` leftovers) | todo |
| T6b | `arms_rviz_control_plugin` panels (FSM/gripper/joint) load in rviz2 | **PASS** — `.so` present in `rviz2`'s `/proc/<pid>/maps`, 0 `PluginlibFactory` errors, `/rviz` node publishes `/fsm_command`, `/hand_controller/target_command`, `/hand_controller/target_percent` | todo |
| T7 | `hardware:=real` — CAN up, `ArxX5Hardware` configure/start succeed, `read()` returns finite joints | **not possible on x86** (§7) | **todo, first thing** |
| T8 | Real arm follows EE target in OCS2 state; gripper open/close over `/hand_controller/target_command` | — | todo |
| T9 | `Ctrl+C` / `stop()` leaves the arm in damping (`set_to_damping`) | — | todo |
| T10 | Every input interface in §9 with mock hardware: `HOME` (reaches `home_pos`), `MOVEJ` + 3-point `JointTrajectory` (reaches last point exactly), `OCS2` + `/left_target`, `/target_path` (3 waypoints, ends at last), `/execute_path` service (`success=true`, duration auto-extended 1.5→2.11 s), gripper 0→1→0 (`gripper_joint` 0→0.044→0), `OCS2↔HOLD` ×3 | **PASS** | todo |
| T11 | RViz marker path replayed as `InteractiveMarkerFeedback`: plain drag publishes nothing (reproduces the "marker does nothing" report); drag + menu `发送目标` → 1 msg on `/left_target/stamped`, arm moves; menu `切换到连续发布` + drag → 22 msgs on `/left_target`, arm follows | **PASS** (see §9.1b) | todo |
| T12 | `/left_target` sent 0.2 / 1.0 / 2.5 / 4.0 s after entering `OCS2` (from `HOME`) — accepted and tracked in all four cases (Δ 0.32–0.90 rad, `/left_current_target` equals the sent pose) | **PASS** | todo |
| T13 | `HOME` entered twice after the `StateHome.cpp` throttle-clock fix: reaches `home_pos` both times, `getting current steady time failed` count 0, throttled WARN printed once | **PASS** | todo |

Probe script used for T2–T4: `e2e_probe.py` (rclpy; waits for controllers, samples
`/joint_states`, switches FSM, publishes target, re-samples, checks marker server).
Re-create it from the description above if needed; it is ~90 lines.

All tests possible without real hardware (T1–T6b) have now passed. See §10 for
timing/resource numbers collected during these runs.

## 7. Known limitations / open items

1. **ARX SDK binaries vs. glibc.** `external/arx5-sdk/lib/x86_64/{libhardware,libsolver}.so`
   need `GLIBC_2.34` / `GLIBCXX_3.4.29` — they cannot load on Ubuntu 20.04 x86_64,
   so the real-hardware path is untestable in this Docker. The **aarch64** builds need
   only `GLIBC_2.17` + `GLIBCXX_3.4.29`: on the Jetson install a GCC 11 libstdc++
   (`ubuntu-toolchain-r/test` PPA, package `libstdc++6`) and it should load.
   Verify with `objdump -T lib*.so | grep -oE 'GLIBC(XX)?_[0-9.]+' | sort -uV`.
2. **No runtime gain tuning** on Foxy (`ros2 param set … joint_k_gains`); edit the
   URDF `<hardware><param>` values instead.
3. `mpc_frequency` is not set in `arx5_description/config/ros2_control/ros2_controllers.yaml`;
   the controller falls back to `update_rate/4 = 12.5 Hz`. Set it explicitly
   (e.g. 50) before hardware tests.
4. Foxy `fake_components` ignores `initial_value` params, so the mock arm starts at
   all-zero joints. Harmless for sim; irrelevant on hardware.
5. `arms_rviz_control_plugin` **is** ported and kept (§4.7) — the FSM/gripper/joint
   panels are available in `demo.rviz`; only the built-in `rviz_common/Time` panel
   had to go (does not exist before Humble). Topic-based control (§5) still works
   as a headless fallback.
6. `ocs2_wbc_controller`, `arx_lift_hardware`, VR/teleop paths are untouched Jazzy code.
7. `robot_descriptions/common` was moved to upstream `origin/main`; if the parent repo
   is committed, update the submodule pointer.
8. Two `git status` noise items: converted `.dae` meshes (untracked, 55 files) and the
   `.glb` originals still present — decide whether to commit both or drop `.glb`.

## 8. Next steps for the Jetson

1. Clone the three repos + submodules (HTTPS), `apt install` per §2, add GCC 11 `libstdc++6`.
2. Build per §5; run T1–T6b with `hardware:=mock_components` to confirm the port on aarch64.
3. Bring up CAN (`can1` per `robot.xacro`), run T7 with the arm free to move, then T8/T9.
4. Tune `joint_k_gains`/`joint_d_gains` in the URDF and `mpc_frequency` in the yaml.

See `foxy.md` (same directory) for the step-by-step Jetson setup/build/run guide
this plan feeds into.

## 9. Feeding the arm a trajectory from an external topic

Everything below was read from the code and confirmed against the live ROS graph
of the running demo (`ros2 topic/service/action list`), so the names are exact.
All names are as seen from the outside; the controller node is `ocs2_arm_controller`.

### 9.1 The FSM gate — nothing is accepted in the wrong state

`ocs2_arm_controller` runs a small FSM: `HOLD` ⇄ `OCS2` / `MOVEJ` / `HOME`.
Cartesian targets are only accepted while the FSM is in **`OCS2`**
(`PoseBasedReferenceManager::accepting_targets_` is flipped by
`Ocs2ArmController::syncPoseTargetAcceptance()` on every transition); joint-space
trajectories are only executed while in **`MOVEJ`**. Messages that arrive in the
wrong state are silently dropped. Transitions go over `/fsm_command`
(`std_msgs/Int32`) and always pass through `HOLD`:

| From | `/fsm_command` | To |
|---|---|---|
| `HOLD` | 1 / 3 / 4 | `HOME` / `OCS2` / `MOVEJ` |
| `OCS2`, `MOVEJ`, `HOME` | 2 | `HOLD` |

Current state is published on `/fsm_state` (`std_msgs/Int32`). So switching
from joint-space streaming to Cartesian streaming is `2` then `3`, and the
reverse is `2` then `4`. (The RViz `OCS2FSMPanel` just publishes these numbers.)

### 9.1b Using the RViz interactive marker (why "dragging does nothing")

`arms_target_manager` puts an interactive marker (`left_arm_target`, frame
`base_link`) on the end effector. **Dragging it only moves the marker** — by
default the manager is in *single-shot* mode (`current_mode_ = SINGLE_SHOT` in
`ArmsTargetManager.h`) and `handleMarkerFeedback()` publishes nothing unless
`shouldStreamPoseCommands()` is true. This was reproduced on the dev box by
replaying `POSE_UPDATE`/`MOUSE_UP` feedback in the `OCS2` state: 0 messages on
`/left_target` and `/left_target/stamped`, joints unchanged. To make the arm
follow, use the marker's **right-click menu**:

| Menu item | What it does | Verified |
|---|---|---|
| `发送目标` | One-shot: publishes the marker pose once on `/left_target/stamped` (frame from `/left_current_target`, else `control_base_frame`). Drag, release, right-click, click this. | yes — joints moved to the dragged pose |
| `切换到连续发布` / `切换到单次发布` | Toggles *continuous* mode: while dragging, the pose streams on `/left_target` at `publish_rate` (20 Hz default). Toggle back when done. | yes — 22 msgs during one drag, arm followed |
| `发送双臂` | dual-arm only, absent on the ARX |

Two more gates that make the marker look dead:
- The FSM must be in `OCS2` (§9.1). The manager tracks the state by listening to
  `/fsm_command` (not `/fsm_state`), so use the FSM panel or `ros2 topic pub /fsm_command`
  — both go through the same topic. In `MOVEJ` (4) continuous mode is forced off.
- After a target is sent, the marker ignores `/left_current_target` for 1 s
  (cool-down), then snaps to the controller's current target — so if you drag in
  single-shot mode and never send, the marker will jump back on the next update.
- There is **no settle time** after entering `OCS2`: a target published 0.2 s, 1 s,
  2.5 s and 4 s after `/fsm_command=3` was accepted and tracked every time (T12).
  "MPC reset complete; waiting for initial policy" in the log is not a reason to wait.

### 9.2 Cartesian interfaces (FSM = `OCS2`) — the end-effector `gripper_center` in `base_link`

| Interface | Type | Semantics |
|---|---|---|
| `/left_target` | `geometry_msgs/Pose` | **Step target**, assumed already in `base_link`. Replaces the reference immediately; the MPC does the smoothing. Publish this at your planner's rate for streaming. |
| `/left_target/stamped` | `geometry_msgs/PoseStamped` | Same, but `header.frame_id` is honoured: if ≠ `base_link` it is TF-transformed (latest transform, `tf2::TimePointZero`) before use. Use this when the pose comes from a camera/world frame. |
| `/left_target/twist` | `geometry_msgs/Twist` | **Velocity streaming** (teleop style). Latched; integrated into the target every control cycle (`integrateLatchedTwists(dt)`); an all-zero twist stops it. |
| `/left_target/relative` | `geometry_msgs/TwistStamped` | One-shot relative displacement added to the current target. |
| `/target_path` | `nav_msgs/Path` | **Multi-waypoint S-curve (moveL)** through every pose in order. **All poses are assumed in `base_link`; `header.frame_id` and per-pose stamps are ignored** (`pathCallback`, "默认所有点都在 base frame 下，不做 TF 转换"). Total duration = `movel_trajectory_duration` param (default 2.0 s), split across segments proportionally to each segment's minimum feasible time under `movel_max_linear/angular_{velocity,acceleration,jerk}`, and auto-extended if that minimum exceeds the requested duration (`movel_auto_extend_duration`, default true). The current target is prepended as the start point. |
| `/execute_path` (service) | `arms_ros2_control_msgs/srv/ExecutePath` | Same machinery as `/target_path`, but with an explicit `trajectory_duration` in the request (0 → param default) and a reply carrying the actual `estimated_duration`. Prefer this over the topic when you need to know when the motion ends. |
| `/ocs2_arm_controller/execute_linear` (action) | `arms_ros2_control_msgs/action/ExecuteLinear` | Straight-line moveL to one `endpoint`, either `time_mode=true` with `duration`, or constraint mode with the `max_*` fields; `frame_id` is honoured. |
| `/ocs2_arm_controller/execute_circle_use_ik` (action) | `.../MovecUseIK` | **Not functional on this branch** — needs `lina_planning`, which is a private submodule that is not built (`lina_planning not available, circle execution services are disabled` in the log). |

`right_target/*` and `dual_target/stamped` exist but are for dual-arm robots; on the
single-arm ARX everything goes through the `left_*` names (the code comments say so).

### 9.3 Joint-space interfaces (FSM = `MOVEJ`)

| Interface | Type | Semantics |
|---|---|---|
| `/ocs2_arm_controller/target_joint_position` | `std_msgs/Float64MultiArray` | Single joint-space target, 6 values in controller joint order (`joint1..joint6`). Interpolated with the `movej_*` params (`movej_duration` 3.0 s, `movej_interpolation_type`, `movej_max_velocity/acceleration/jerk`). |
| `/ocs2_arm_controller/target_joint_trajectory` | `trajectory_msgs/JointTrajectory` | Multi-point joint trajectory. **Joints are matched by name** (`joint_names`), a subset is fine — unlisted joints are held. **`time_from_start` is ignored**: the whole trajectory is stretched over `movej_trajectory_duration` (default 3.0 s) with segment durations derived from that total (`calculateSegmentDurations`), unless `lina_planning` is present. Needs **≥ 2 points** (the current position is prepended, and the planner wants ≥ 3). Positions are clamped to URDF limits. |
| `/ocs2_arm_controller/joint_trajectory_with_para` (action) | `arms_ros2_control_msgs/action/JointTrajectory` | Same, with per-waypoint `JointWaypoint` carrying `max_velocity/acceleration/jerk`, `blend_ratio_percent`, and `time_mode`/`total_time` — this is the interface to use if you need control over timing. Feedback: `progress`, `elapsed_time`, `remaining_time`. |

### 9.4 What to pick for "an external node produces the trajectory"

- **Reactive / closed-loop planner (re-plans at 10–100 Hz, e.g. visual servoing, teleop, RL policy):**
  go to `OCS2` (`/fsm_command=3`) and publish `/left_target/stamped` (or
  `/left_target`) at the planner rate. Each message is a fresh setpoint; the MPC
  (`mpc_frequency`, see §7.3) turns the stream into a smooth, constraint-respecting
  motion. For velocity-command planners use `/left_target/twist` instead. This is the
  lowest-latency path and the one the OCS2 stack is designed around.
- **Pre-computed Cartesian path (a list of EE poses, executed once):**
  `OCS2` state, then either `/target_path` (fire-and-forget) or the `/execute_path`
  service (blocking, returns duration). Remember the poses must already be in
  `base_link` and the timing you get is the S-curve's, not yours.
- **Pre-computed joint-space trajectory (e.g. from MoveIt/OMPL, a recorded demo, or
  an offline optimizer):** `MOVEJ` state (`/fsm_command=4`), then
  `/ocs2_arm_controller/target_joint_trajectory` for simple cases, or the
  `joint_trajectory_with_para` action when you care about per-segment timing.
  Note this path bypasses the MPC entirely — no self-collision or EE constraints are
  enforced beyond URDF joint limits.

Minimal example, Cartesian stream:

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"
# then from your node, at e.g. 50 Hz:
#   geometry_msgs/PoseStamped  header.frame_id="base_link" (or any TF-connected frame)
#   -> topic /left_target/stamped
```

Minimal example, joint trajectory:

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 4}"
ros2 topic pub --once /ocs2_arm_controller/target_joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
points:
- {positions: [0.0, 0.5, 0.3, 0.3, 0.0, 0.0]}
- {positions: [0.4, 0.6, 0.2, 0.3, 0.2, 0.0]}"
```

### 9.5 Gaps you may hit, and where the fix would go

1. **Timestamps are ignored on both `nav_msgs/Path` and `JointTrajectory`.** If your
   external planner needs its own time parameterization honoured, the smallest
   change is in `PoseBasedReferenceManager::setArmReferenceBufferFromWaypoints()`
   (`ocs2_controller_common`) — derive `buffer.segmentDurations` from the pose
   stamps instead of the `movel_*` limits — and in
   `StateMoveJ::calculateSegmentDurations()` (`arms_controller_common`) for the
   joint case. Until then, the workaround is to time-slice on the sender side and
   stream `/left_target/stamped` yourself.
2. **`/target_path` does no TF.** Transform on the sender, or send each pose through
   `/left_target/stamped`, or add the same `parsePoseStampedToState()` call that
   `leftPoseStampedCallback` already uses into `pathCallback`.
3. **No feedback topic for "target reached"** on the Cartesian topics; poll
   `/left_current_target` (`PoseStamped`, the EE pose the controller is currently
   commanding) or use the service/action variants, which reply.
4. **Anything sent while not in the matching FSM state is dropped without a log line.**
   Check `/fsm_state` before publishing.

## 10. Profiling (x86_64 Docker, 28 cores / 62 GB RAM — a ceiling, not a Jetson estimate)

Collected on the mock-hardware demo (`robot:=arx5 hardware:=mock_components`).
Numbers are here to give the Jetson session something concrete to compare against,
not as a guarantee — a Jetson has far fewer, far slower cores and no host cache
warm from a previous run.

**Build** (clean `rm -rf build install log`, `--parallel-workers` default = nproc):
- 29 packages, 0 errors, **2m07s** wall clock.
- Heaviest individual packages: `ocs2_core` 34.6s, `arms_target_manager` 26.6s,
  `arms_controller_common` 22.2s, `ocs2_mobile_manipulator_ros` 21.5s,
  `ocs2_arm_controller` 20.0s (each package's own single-threaded portion; colcon
  runs independent packages in parallel, so wall clock ≪ sum). On a Jetson with
  4–12 cores, expect the total to land somewhere between "sum of the above" and
  "2m07s × (28 / Jetson core count)" — budget 15–40 minutes for a clean build and
  do **not** run it in parallel with anything else memory-hungry (`ocs2_core`,
  `arms_target_manager`, and `arx_ros2_control`'s vendored SDK compile are the
  ones most likely to OOM a Jetson Nano/Orin Nano's 4–8 GB if colcon parallelism
  is left at `nproc`; pass `--parallel-workers 2` if RAM is tight).

**Launch cold-start** (CppAD codegen cache empty — `rm -rf ~/.ros/ocs2_cache /tmp/ocs2_ros2`
before launch):
- `demo.launch.py` → all three controllers (`joint_state_broadcaster`,
  `hand_controller`, `ocs2_arm_controller`) reach `active`: **3.1s**.
  Only 3 small CppAD libraries (`dynamics_flow_map`, `dynamics_jump_map`,
  `dynamics_guard_surfaces` — a few KB each) are compiled at this stage; the
  end-effector and self-collision constraint terms use Pinocchio's analytical
  Jacobians directly and need no codegen at all for this robot, so there is no
  further compile step hiding behind the first `HOLD → OCS2` FSM transition
  either (measured at **1.9s**, dominated by MPC reset, not compilation). The
  once-feared "up to a minute of silence" does not happen for a 6-DOF fixed-base
  arm on this hardware; it may still apply to robots with self-collision terms
  large enough to need their own CppAD codegen, or on a much slower CPU.

**Control loop timing** (`ros2 param set /ocs2_arm_controller rt_timing_enabled true`,
50 Hz `update_rate`, i.e. a 20,000 µs period budget), steady state in the `ocs2` FSM
state, ~10 s / ~1000 cycles sampled:
- `update()` total: p50 = 10–12 µs, p99 = 28–48 µs, max = 30–52 µs.
- Breakdown (p50): observation read 2–5 µs, EE forward-kinematics 1–3 µs,
  MPC policy evaluation 1–3 µs, everything else (viz scheduling, command write,
  MRT bookkeeping) ≤ 1 µs each.
- Headroom on this box is enormous (worst case ~52 µs against a 20,000 µs budget,
  a >99.7% margin) — this says nothing about Jetson margins and should be
  re-measured there with the same `rt_timing_enabled` flag; a Jetson Orin Nano's
  single MPC solve is a very different cost profile than this idle-loop number
  suggests, since the heavy solve happens in the separate MPC thread
  (`mpcUpdateThread`), not in `update()` — profile that thread's cadence too by
  watching `ros2 topic hz /mpc_observation` or similar once on hardware.

**Process memory (RSS)**, full demo running, mock hardware, RViz with all
displays + 3 panels visible:
- `rviz2`: ~280 MB (dominated by Qt/Ogre, not this port's changes)
- `ros2_control_node` (hosts `ocs2_arm_controller` + `hand_controller` +
  `joint_state_broadcaster`, including the loaded Pinocchio model and MPC/DDP
  state): ~66 MB
- `robot_state_publisher`: ~25 MB
- `arms_target_manager_node`: ~38 MB
- Total for the whole demo minus RViz: well under 200 MB — should fit
  comfortably even on an 4 GB Jetson Orin Nano; RViz itself is the only piece
  worth watching on a memory-constrained board (headless operation, i.e.
  `rviz:=false`, avoids it entirely).
