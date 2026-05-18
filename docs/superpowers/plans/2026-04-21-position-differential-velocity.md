# Position Differential Velocity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute joint velocity from position changes instead of reading PRESENT_SPEED register (which returns 0 due to RS485 timing issues).

**Architecture:** Store previous position and timestamp per joint in JointManager. On each successful position read, compute velocity as `(delta_pos * 360.0 / 4096.0) / delta_sec`. Velocity output matches position output in degrees/second.

**Tech Stack:** C++17, ROS 2, tachimawari, standard `<chrono>` library

---

## File Structure

| File | Change |
|------|--------|
| `include/tachimawari/joint/node/joint_manager.hpp` | Add `joint_read_state` map and `compute_velocity_from_differential()` method |
| `src/tachimawari/joint/node/joint_manager.cpp` | Implement velocity computation, remove PRESENT_SPEED_L read |
| `src/tachimawari/joint/node/joint_node.cpp` | Pass timestamp to JointManager when publishing |

---

## Task 1: Add JointManager members

**Files:**
- Modify: `include/tachimawari/joint/node/joint_manager.hpp:21-29`

- [ ] **Step 1: Add includes for chrono and unordered_map**

Add after line 24 (`#include <vector>`):

```cpp
#include <chrono>
#include <unordered_map>
```

- [ ] **Step 2: Add joint_read_state member variable**

Add after line 51 (`std::vector<Joint> current_joints;`):

```cpp
// Stores previous raw position value (ticks) and timestamp for differential velocity
std::unordered_map<uint8_t, std::pair<int, std::chrono::steady_clock::time_point>> joint_read_state;
```

- [ ] **Step 3: Add compute_velocity_from_differential method declaration**

Add after line 47 (`void update_current_joints_from_control_manager(const std::vector<Joint> & joints);`):

```cpp
void compute_velocity_from_differential(
  uint8_t id, int new_position, const std::chrono::steady_clock::time_point & now);
```

---

## Task 2: Implement velocity computation in JointManager

**Files:**
- Modify: `src/tachimawari/joint/node/joint_manager.cpp`

- [ ] **Step 1: Add cmath include**

Add after line 24 (`#include <cmath>`):

```cpp
#include <chrono>
```

- [ ] **Step 2: Add compute_velocity_from_differential implementation**

Add after line 60 (after `update_current_joints()` method):

```cpp
void JointManager::compute_velocity_from_differential(
  uint8_t id, int new_position, const std::chrono::steady_clock::time_point & now)
{
  auto it = joint_read_state.find(id);
  if (it != joint_read_state.end()) {
    auto [prev_value, prev_time] = it->second;
    double delta_pos = new_position - prev_value;
    double delta_sec = std::chrono::duration<double>(now - prev_time).count();

    if (delta_sec > 0) {
      // Dynamixel MX: 4096 ticks per rotation, 360 degrees per rotation
      double degrees_per_tick = 360.0 / 4096.0;
      double velocity = (delta_pos * degrees_per_tick) / delta_sec;

      // Set velocity on the corresponding joint
      for (auto & joint : current_joints) {
        if (joint.get_id() == id) {
          joint.set_velocity(static_cast<float>(velocity));
          break;
        }
      }
    }
  }
  // Always update state with new position and time
  joint_read_state[id] = {new_position, now};
}
```

- [ ] **Step 3: Modify update_current_joints_from_control_manager to use differential velocity**

Replace lines 62-87 with:

```cpp
void JointManager::update_current_joints_from_control_manager(const std::vector<Joint> & joints)
{
  std::vector<Joint> new_joints(joints);
  auto now = std::chrono::steady_clock::now();

  for (auto & joint : new_joints) {
    int current_value = control_manager->read_packet(
      joint.get_id(), tachimawari::joint::protocol_1::MX28Address::PRESENT_POSITION_L, 2);

    if (current_value != -1) {
      joint.set_position_value(current_value);
      compute_velocity_from_differential(joint.get_id(), current_value, now);
    } else {
      // Keep previous position value on read failure
      joint.set_position_value(joint.get_position_value());
    }
  }

  update_current_joints(new_joints);
}
```


---

## Task 3: Pass timestamp from JointNode

**Files:**
- Modify: `src/tachimawari/joint/node/joint_node.cpp:85-99`

- [ ] **Step 1: Modify publish_current_joints to pass timestamp**

The `publish_current_joints()` method calls `joint_manager->get_current_joints()`. We need JointManager to accept a timestamp parameter for consistent velocity computation across all joints.

Add a new overloaded `get_current_joints` method that accepts timestamp:

In `joint_manager.hpp`, add:

```cpp
const std::vector<Joint> & get_current_joints(
  const std::chrono::steady_clock::time_point & now);
```

In `joint_manager.cpp`, add:

```cpp
const std::vector<Joint> & JointManager::get_current_joints(
  const std::chrono::steady_clock::time_point & now)
{
  if (!is_each_joint_updated) {
    update_current_joints_from_control_manager(current_joints);
  }
  return current_joints;
}
```

Then modify `joint_node.cpp` `publish_current_joints()`:

```cpp
void JointNode::publish_current_joints()
{
  auto now = std::chrono::steady_clock::now();
  const auto & current_joints = this->joint_manager->get_current_joints(now);
  // ... rest unchanged
}
```

---

## Task 4: Verify build

- [ ] **Step 1: Build the package**

```bash
cd /home/usb/Desktop/ichiro/ichiro-ws && colcon build --packages-select tachimawari
```

Expected: Build succeeds with no errors

- [ ] **Step 2: Verify no linter issues**

```bash
cd /home/usb/Desktop/ichiro/ichiro-ws/src/tachimawari && clang-format --Werror --dry-run src/tachimawari/joint/node/joint_manager.cpp src/tachimawari/joint/node/joint_node.cpp
```

---

## Verification Checklist

- [ ] Joint.msg already has `uint8 id`, `float32 position`, `float32 velocity` fields (no msg changes needed)
- [ ] `/joint/current_joints` topic publishes velocity in degrees/second
- [ ] Velocity computed only when position read succeeds
- [ ] Velocity goes to 0 when servo is stationary
- [ ] First read initializes state without computing velocity (no division by zero)
