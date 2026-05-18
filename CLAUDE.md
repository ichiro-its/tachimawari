# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This is a ROS 2 package using `ament_cmake`. All commands run from the workspace root (e.g. `~/Desktop/ichiro/ichiro-ws`).

```bash
# Build
colcon build --packages-select tachimawari

# Build with dependencies
colcon build --packages-select tachimawari tachimawari_interfaces kansei_interfaces keisan

# Run tests
colcon test --packages-select tachimawari && colcon test-result --verbose

# Lint
ament_lint_auto  # from package root, runs ament_lint_common checks

# Single test file
colcon build --packages-select tachimawari && ament test --runtime-dir build/tachimawari test/joint/joint_test.cpp
```

## Architecture

### Layered Overview

```
TachimawariNode
├── JointNode      → publishes /joint/current_joints (and vel)
├── ImuNode        → publishes IMU data
├── ControlNode    → handles control instructions
└── ControlManager (virtual base)
    └── Controller (CM740 + Linux platform) → DYNAMIXEL SDK
```

**TachimawariNode** (`node/tachimawari_node`) is the top-level ROS 2 node. It owns `JointNode`, `ImuNode`, `ControlNode`, and a `ControlManager`. It runs the joint manager loop and IMU provider loop via timers.

**JointManager** (`joint/node/joint_manager`) is the business logic layer for joints. It caches current joint state, writes target positions via `ControlManager`, and reads back current positions. It owns the velocity differential state.

**ControlManager** (`control/manager/control_manager`) is a virtual interface for DYNAMIXEL communication. Subclasses implement `write_packet`, `read_packet`, `sync_write_packet`, and bulk read/write operations. Device IDs are defined as `MARIN_CORE = 190`, `CONTROLLER = 200`, `BROADCAST = 254`.

**Controller** (`control/controller`) wraps the CM740 module on Linux, bridging to the DYNAMIXEL SDK.

### Protocols

Two DYNAMIXEL protocol versions are supported:
- **Protocol 1**: MX28 servos — register addresses in `joint/protocol_1/mx28_address.hpp` and `control/controller/module/cm740_address.hpp`
- **Protocol 2**: higher-performance servos — addresses in `joint/protocol_2/mx28_address.hpp`

SDK wrappers exist under `control/sdk/packet/protocol_1/` and `control/sdk/packet/protocol_2/` for group bulk read and group sync write.

### Key Files

- `CMakeLists.txt` lists all library sources explicitly — adding a new `.cpp` requires adding it here manually
- Standalone executables (`check_joints`, `init_joints`, `read_joints`, `read_buttons`, `main`) are built in `CMakeLists.txt` and linked against the shared library
- `Joint` model (`joint/model/joint.hpp`) holds id, position (radians), velocity (rad/s), and torque state
- `JointId` (`joint/model/joint_id.hpp`) defines the servo ID list

### Joint Address Map

DYNAMIXEL MX28 registers used:
- `PRESENT_POSITION_L/H` (address 37, 38) — current position in ticks (Protocol 1)
- `PRESENT_SPEED_L/H` (address 39, 40) — current speed (often returns 0 due to RS485 timing; differential velocity from position is the preferred approach)
- `GOAL_POSITION_L/H` (address 42, 43) — target position
- `GOAL_VELOCITY_L/H` (address 44, 45) — target velocity

### Velocity Computation

Joint velocity is computed by `JointManager` using position differential with a timestamp map (`joint_read_state`). On each position read, it computes `(delta_ticks * 360/4096) / delta_sec` in degrees/second. The first read initializes state without computing velocity (avoids division by zero).

## Branch Naming Convention

```
feature/JIRA-ID-SHORT-DESCRIPTION  # with JIRA ticket
enhancement/SHORT-DESCRIPTION     # enhancement, with or without ticket
hotfix/SHORT-DESCRIPTION          # urgent, no testing needed
```

## Dependencies

This package depends on sibling ROS 2 packages in the same workspace:
- `tachimawari_interfaces` — ROS msg/srv definitions
- `kansei_interfaces` — sensor interfaces
- `keisan` — arithmetic utilities
- `jitsuyo` — general utilities
- `dynamixel_sdk` — low-level DYNAMIXEL protocol library
- `rclcpp`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`

When building locally, ensure all sibling dependencies are built first.

## In-Progress Design Docs

`docs/superpowers/` contains implementation plans for active work (joint velocity publisher, position differential velocity). These are authoritative for pending feature work.
