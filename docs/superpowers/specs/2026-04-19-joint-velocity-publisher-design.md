# Joint Velocity Publisher Design

**Date:** 2026-04-19

## Overview

Add a joint velocity publisher to `tachimawari` package that publishes current joint velocities in rad/s alongside the existing position publisher.

## Message Changes

### tachimawari_interfaces package

| Action | File |
|--------|------|
| Rename | `msg/CurrentJoints.msg` → `msg/CurrentJointsPos.msg` |
| Create | `msg/CurrentJointsVel.msg` (structure: `Joint[] joints`) |
| Update | `CMakeLists.txt` |

**CurrentJointsVel.msg:**
```
Joint[] joints
```

### Topic Names

| Old | New |
|-----|-----|
| `joint/current_joints` | `joint/current_joints_pos` |
| (new) | `joint/current_joints_vel` |

## Implementation

### Velocity Reading

- Read raw `PRESENT_SPEED_L/H` (addresses 38-39) from DYNAMIXEL MX28 servos
- DYNAMIXEL speed format: 10-bit RPM with lowest bit as direction (0=CW, 1=CCW)
- Conversion to rad/s: `abs(raw_speed) * (2π / 60)`

### Files to Modify

1. `tachimawari_interfaces/msg/CurrentJoints.msg` → `CurrentJointsPos.msg`
2. `tachimawari_interfaces/CMakeLists.txt`
3. `include/tachimawari/joint/node/joint_node.hpp`
4. `src/tachimawari/joint/node/joint_node.cpp`
5. `include/tachimawari/joint/node/joint_manager.hpp`
6. `src/tachimawari/joint/node/joint_manager.cpp`

### JointNode changes

- Add `CurrentJointsVel` type alias
- Add `current_joints_vel_topic()` static method
- Add `current_joints_vel_publisher` member
- Add `publish_current_joints_vel()` method

### JointManager changes

- Add method to read velocity from control manager
- Add `get_current_joints_velocity()` that returns vector of velocities in rad/s

## Implementation Notes

- Velocity values are signed: positive for CCW, negative for CW
- Publish on same cycle as position publisher
- Velocity reading uses existing `control_manager->read_packet()` pattern