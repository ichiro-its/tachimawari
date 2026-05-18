# Joint Velocity from Position Differential Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On each joint position read, compute velocity from the change in raw tick position over elapsed time and publish it via the existing `/joint/current_joints` topic (which already has a `velocity` field in `Joint.msg`).

**Architecture:** `JointManager` tracks the previous raw tick position and timestamp per joint. After reading a new position, it computes `velocity_deg_s = (delta_ticks * 360.0 / 4096.0) / delta_sec`. `Joint::set_position()` is extended with a velocity setter, so `JointNode::publish_current_joints()` only needs to copy the velocity field into the message. No new publishers or message types needed.

**Tech Stack:** C++17, `<chrono>`, ROS 2, tachimawari, tachimawari_interfaces

---

## File Structure

| File | Change |
|------|--------|
| `include/tachimawari/joint/model/joint.hpp` | Add velocity member + getter/setter |
| `src/tachimawari/joint/model/joint.cpp` | Implement setter/getter |
| `include/tachimawari/joint/node/joint_manager.hpp` | Add `joint_read_state` map |
| `src/tachimawari/joint/node/joint_manager.cpp` | Implement differential velocity computation |
| `src/tachimawari/joint/node/joint_node.cpp` | Copy velocity into `CurrentJoints` message |

---

## Task 1: Add velocity to the Joint model

**Files:**
- Modify: `include/tachimawari/joint/model/joint.hpp:44-66`
- Modify: `src/tachimawari/joint/model/joint.cpp:40-99`

- [ ] **Step 1: Add velocity getter/setter declarations to Joint.hpp**

After `float get_position() const;` (line 54), add:

```cpp
  void set_velocity(float velocity);
  float get_velocity() const;
```

After `keisan::Angle<float> position;` (line 66), add the private member:

```cpp
  float velocity;
```

- [ ] **Step 2: Initialize velocity to 0 in both constructors in Joint.cpp**

In `Joint::Joint(uint8_t joint_id, float position)` (line 40), change the initializer to:

```cpp
Joint::Joint(uint8_t joint_id, float position)
: id(joint_id), position(keisan::make_degree(position)), velocity(0.0)
```

In `Joint::Joint(uint8_t joint_id, keisan::Angle<float> position)` (line 52), change to:

```cpp
Joint::Joint(uint8_t joint_id, keisan::Angle<float> position)
: Joint(joint_id, position.degree())
```

- [ ] **Step 3: Implement set_velocity and get_velocity in Joint.cpp**

After `float Joint::get_position() const` (line 79-82), add:

```cpp
void Joint::set_velocity(float velocity)
{
  this->velocity = velocity;
}

float Joint::get_velocity() const
{
  return velocity;
}
```

---

## Task 2: Add differential velocity state to JointManager

**Files:**
- Modify: `include/tachimawari/joint/node/joint_manager.hpp:21-53`

- [ ] **Step 1: Add chrono and unordered_map includes**

After `#include <vector>` (line 26), add:

```cpp
#include <chrono>
#include <unordered_map>
```

- [ ] **Step 2: Add joint_read_state member**

After `std::vector<Joint> current_joints;` (line 51), add:

```cpp
  // Maps joint id → {previous tick value, previous timestamp}
  std::unordered_map<uint8_t, std::pair<int, std::chrono::steady_clock::time_point>>
    joint_read_state;
```

- [ ] **Step 3: Commit**

---

## Task 3: Implement velocity computation in JointManager

**Files:**
- Modify: `src/tachimawari/joint/node/joint_manager.cpp:21-120`

- [ ] **Step 1: Add chrono include**

After `#include <algorithm>` (line 23), add:

```cpp
#include <chrono>
```

- [ ] **Step 2: Add differential velocity computation after update_current_joints**

In `update_current_joints_from_control_manager`, after each successful `read_packet` call (after `joint.set_position_value(value);`, line 71), add a new helper method call.

First, add a new method declaration to `joint_manager.hpp` after `void update_current_joints_from_control_manager(const std::vector<Joint> & joints);` (line 47):

```cpp
  void compute_velocity_from_differential(uint8_t id, int new_position);
```

Then add the implementation before `get_current_joints()` in `joint_manager.cpp`. Add it after `update_current_joints_from_control_manager` closes and before `get_current_joints` (around line 76):

```cpp
void JointManager::compute_velocity_from_differential(uint8_t id, int new_position)
{
  auto it = joint_read_state.find(id);
  auto now = std::chrono::steady_clock::now();

  if (it != joint_read_state.end()) {
    int prev_value = it->second.first;
    auto prev_time = it->second.second;
    double delta_sec = std::chrono::duration<double>(now - prev_time).count();

    if (delta_sec > 0) {
      // Dynamixel MX28: 4096 ticks per rotation, 360 degrees per rotation
      int raw_delta = new_position - prev_value;
      // Handle wraparound: if delta is more than half a rotation, it wrapped
      if (raw_delta > 2048) {
        raw_delta -= 4096;
      } else if (raw_delta < -2048) {
        raw_delta += 4096;
      }

      double velocity_deg_s = (raw_delta * 360.0 / 4096.0) / delta_sec;
      for (auto & joint : current_joints) {
        if (joint.get_id() == id) {
          joint.set_velocity(static_cast<float>(velocity_deg_s));
          break;
        }
      }
    }
  }

  // Always update state with new position and time
  joint_read_state[id] = {new_position, now};
}
```

- [ ] **Step 3: Call compute_velocity_from_differential in update_current_joints_from_control_manager**

In `update_current_joints_from_control_manager` (line 66-72), after `joint.set_position_value(value);` (line 71), add:

```cpp
    compute_velocity_from_differential(joint.get_id(), value);
```

The updated loop body should be:

```cpp
  for (auto & joint : new_joints) {
    float value = Joint::CENTER_VALUE;

    int current_value =
      control_manager->read_packet(joint.get_id(), protocol_1::MX28Address::PRESENT_POSITION_L, 2);

    value = (current_value == -1) ? value : current_value;

    joint.set_position_value(value);
    compute_velocity_from_differential(joint.get_id(), value);
  }
```

---

## Task 4: Publish velocity in JointNode

**Files:**
- Modify: `src/tachimawari/joint/node/joint_node.cpp:85-98`

- [ ] **Step 1: Add velocity to the published message in publish_current_joints**

In `publish_current_joints()` (lines 85-98), add `joints[i].velocity = current_joints[i].get_velocity();` after `joints[i].position = current_joints[i].get_position();` (line 94):

```cpp
void JointNode::publish_current_joints()
{
  const auto & current_joints = this->joint_manager->get_current_joints();
  auto msg_joints = CurrentJoints();
  auto & joints = msg_joints.joints;

  joints.resize(current_joints.size());
  for (size_t i = 0; i < joints.size() && i < current_joints.size(); ++i) {
    joints[i].id = current_joints[i].get_id();
    joints[i].position = current_joints[i].get_position();
    joints[i].velocity = current_joints[i].get_velocity();
  }

  current_joints_publisher->publish(msg_joints);
}
```

---

## Task 5: Build and verify

- [ ] **Step 1: Build the package**

Run from workspace root:
```bash
cd /home/usb/Desktop/ichiro/ichiro-ws && colcon build --packages-select tachimawari
```
Expected: build succeeds with no errors or warnings

- [ ] **Step 2: Run tests**

```bash
cd /home/usb/Desktop/ichiro/ichiro-ws && colcon test --packages-select tachimawari && colcon test-result --verbose
```
Expected: all tests pass

- [ ] **Step 3: Format check**

```bash
cd /home/usb/Desktop/ichiro/ichiro-ws/src/tachimawari && clang-format --Werror --dry-run \
  src/tachimawari/joint/model/joint.cpp \
  src/tachimawari/joint/node/joint_manager.cpp \
  src/tachimawari/joint/node/joint_node.cpp
```
Expected: no formatting errors

---

## Verification Checklist

- [ ] `Joint.msg` field `float32 velocity` is already present (no msg changes needed)
- [ ] `/joint/current_joints` topic now carries velocity in degrees/second per servo
- [ ] First position read per joint initializes state without computing velocity (no division by zero)
- [ ] Servo wraparound across 0/4095 ticks is handled (delta normalization)
- [ ] Velocity is 0 when servo is stationary (delta_ticks ≈ 0 → velocity ≈ 0)
