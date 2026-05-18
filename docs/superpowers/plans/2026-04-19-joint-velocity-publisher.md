# Joint Velocity Publisher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add joint velocity publisher to tachimawari package that publishes current joint velocities in rad/s alongside existing position publisher.

**Architecture:** Read PRESENT_SPEED from DYNAMIXEL MX28 servos, convert to rad/s, publish via new ROS 2 publisher with renamed topic. Messages renamed from CurrentJoints to CurrentJointsPos/CurrentJointsVel.

**Tech Stack:** ROS 2, C++, DYNAMIXEL SDK, tachimawari_interfaces

---

## Task 1: Rename CurrentJoints message

**Files:**
- Rename: `tachimawari_interfaces/msg/CurrentJoints.msg` → `tachimawari_interfaces/msg/CurrentJointsPos.msg`
- Modify: `tachimawari_interfaces/CMakeLists.txt`

- [ ] **Step 1: Rename message file**

```bash
mv tachimawari_interfaces/msg/CurrentJoints.msg tachimawari_interfaces/msg/CurrentJointsPos.msg
```

- [ ] **Step 2: Update CMakeLists.txt**

Edit `tachimawari_interfaces/CMakeLists.txt` line 21:
```cmake
  "msg/CurrentJointsPos.msg"
```

- [ ] **Step 3: Commit**

```bash
git add tachimawari_interfaces/msg/CurrentJointsPos.msg tachimawari_interfaces/CMakeLists.txt
git commit -m "refactor: rename CurrentJoints to CurrentJointsPos"
```

---

## Task 2: Create CurrentJointsVel message

**Files:**
- Create: `tachimawari_interfaces/msg/CurrentJointsVel.msg`
- Modify: `tachimawari_interfaces/CMakeLists.txt`

- [ ] **Step 1: Create velocity message file**

```bash
cat > tachimawari_interfaces/msg/CurrentJointsVel.msg << 'EOF'
Joint[] joints
EOF
```

- [ ] **Step 2: Update CMakeLists.txt**

Add to `rosidl_generate_interfaces()` in `tachimawari_interfaces/CMakeLists.txt`:
```cmake
  "msg/CurrentJointsVel.msg"
```

- [ ] **Step 3: Commit**

```bash
git add tachimawari_interfaces/msg/CurrentJointsVel.msg tachimawari_interfaces/CMakeLists.txt
git commit -m "feat: add CurrentJointsVel message for joint velocity"
```

---

## Task 3: Update JointNode header

**Files:**
- Modify: `include/tachimawari/joint/node/joint_node.hpp`

- [ ] **Step 1: Add velocity topic method**

Add to `static std::string` section (after line 54):
```cpp
  static std::string current_joints_vel_topic();
```

- [ ] **Step 2: Add velocity publisher member**

Add after `rclcpp::Publisher<CurrentJoints>::SharedPtr current_joints_publisher;` (line 66):
```cpp
  rclcpp::Publisher<tachimawari_interfaces::msg::CurrentJointsVel>::SharedPtr current_joints_vel_publisher;
```

- [ ] **Step 3: Add type alias for CurrentJointsVel**

Add after line 47:
```cpp
  using CurrentJointsVel = tachimawari_interfaces::msg::CurrentJointsVel;
```

- [ ] **Step 4: Commit**

```bash
git add include/tachimawari/joint/node/joint_node.hpp
git commit -m "feat: add velocity publisher declaration to JointNode"
```

---

## Task 4: Update JointNode source

**Files:**
- Modify: `src/tachimawari/joint/node/joint_node.cpp`

- [ ] **Step 1: Add velocity topic method**

Add after `JointNode::current_joints_topic()` (around line 47):
```cpp
std::string JointNode::current_joints_vel_topic() { return get_node_prefix() + "/current_joints_vel"; }
```

- [ ] **Step 2: Update CurrentJoints to CurrentJointsPos in topic method**

Change line 47 from:
```cpp
std::string JointNode::current_joints_topic() { return get_node_prefix() + "/current_joints"; }
```
to:
```cpp
std::string JointNode::current_joints_topic() { return get_node_prefix() + "/current_joints_pos"; }
```

- [ ] **Step 3: Update type alias in publish_current_joints**

Change `CurrentJoints` to `CurrentJointsPos` (line 88):
```cpp
  auto msg_joints = CurrentJointsPos();
```

- [ ] **Step 4: Create velocity publisher in constructor**

Add after `current_joints_publisher = node->create_publisher<CurrentJointsPos>(current_joints_topic(), 10);` (line 80):
```cpp
  current_joints_vel_publisher = node->create_publisher<CurrentJointsVel>(current_joints_vel_topic(), 10);
```

- [ ] **Step 5: Add publish_current_joints_vel method**

Add after `publish_current_joints()` (after line 98):
```cpp
void JointNode::publish_current_joints_vel()
{
  const auto & current_joints_vel = this->joint_manager->get_current_joints_velocity();
  auto msg_vel = CurrentJointsVel();
  auto & joints = msg_vel.joints;

  joints.resize(current_joints_vel.size());
  for (size_t i = 0; i < joints.size() && i < current_joints_vel.size(); ++i) {
    joints[i].id = current_joints_vel[i].first;
    joints[i].position = current_joints_vel[i].second;
  }

  current_joints_vel_publisher->publish(msg_vel);
}
```

- [ ] **Step 6: Commit**

```bash
git add src/tachimawari/joint/node/joint_node.cpp
git commit -m "feat: add velocity publishing to JointNode"
```

---

## Task 5: Update JointManager header

**Files:**
- Modify: `include/tachimawari/joint/node/joint_manager.hpp`

- [ ] **Step 1: Add get_current_joints_velocity method**

Add after `get_current_joints()` (line 43):
```cpp
  const std::vector<std::pair<uint8_t, float>> & get_current_joints_velocity();
```

- [ ] **Step 2: Add velocity joints member**

Add after `std::vector<Joint> current_joints;` (line 51):
```cpp
  std::vector<std::pair<uint8_t, float>> current_joints_vel;
```

- [ ] **Step 3: Commit**

```bash
git add include/tachimawari/joint/node/joint_manager.hpp
git commit -m "feat: add velocity storage to JointManager"
```

---

## Task 6: Update JointManager source

**Files:**
- Modify: `src/tachimawari/joint/node/joint_manager.cpp`

- [ ] **Step 1: Initialize velocity vector in constructor**

Add after initialization loop in constructor (after line 40):
```cpp
  for (auto id : JointId::list) {
    current_joints_vel.push_back(std::make_pair(id, 0.0f));
  }
```

- [ ] **Step 2: Add update method for velocities**

Add before `get_current_joints()` (around line 77):
```cpp
void JointManager::update_current_joints_velocity()
{
  for (auto & vel : current_joints_vel) {
    int raw_velocity = control_manager->read_packet(vel.first, protocol_1::MX28Address::PRESENT_SPEED_L, 2);
    if (raw_velocity != -1) {
      int speed_value = raw_velocity & 0x3FF;
      bool direction = raw_velocity & 0x400;
      float rpm = static_cast<float>(speed_value);
      vel.second = direction ? rpm : -rpm;
      vel.second *= (2.0f * M_PI / 60.0f);
    }
  }
}
```

- [ ] **Step 3: Update get_current_joints to call update_current_joints_velocity**

Add inside `get_current_joints()` after `if (!is_each_joint_updated) {` (around line 79):
```cpp
    update_current_joints_from_control_manager(current_joints);
    update_current_joints_velocity();
```

- [ ] **Step 4: Add get_current_joints_velocity method**

Add after `get_current_joints()` (around line 84):
```cpp
const std::vector<std::pair<uint8_t, float>> & JointManager::get_current_joints_velocity()
{
  if (!is_each_joint_updated) {
    update_current_joints_from_control_manager(current_joints);
    update_current_joints_velocity();
  }

  return current_joints_vel;
}
```

- [ ] **Step 5: Include cmath for M_PI**

Add at top of file after existing includes:
```cpp
#include <cmath>
```

- [ ] **Step 6: Commit**

```bash
git add src/tachimawari/joint/node/joint_manager.cpp
git commit -m "feat: implement velocity reading from DYNAMIXEL servos"
```

---

## Task 7: Update tachimawari_node to call velocity publisher

**Files:**
- Modify: `src/tachimawari/node/tachimawari_node.cpp`

- [ ] **Step 1: Call publish_current_joints_vel in update loop**

Find the timer callback or update loop where `publish_current_joints()` is called and add:
```cpp
joint_node->publish_current_joints_vel();
```

- [ ] **Step 2: Commit**

```bash
git add src/tachimawari/node/tachimawari_node.cpp
git commit -m "feat: publish velocity in tachimawari_node update loop"
```

---

## Task 8: Build and verify

- [ ] **Step 1: Source ROS environment and build**

```bash
cd /home/usb/Desktop/ichiro/ichiro-ws
source install/setup.bash
colcon build --packages-select tachimawari_interfaces
colcon build --packages-select tachimawari
```

- [ ] **Step 2: Verify messages are generated**

```bash
ros2 interface list | grep -E "CurrentJoints"
```

Expected output should show both `tachimawari_interfaces/msg/CurrentJointsPos` and `tachimawari_interfaces/msg/CurrentJointsVel`

- [ ] **Step 3: Commit final verification**

```bash
git status
git log --oneline -5
```