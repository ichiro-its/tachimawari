# Position Differential Velocity Design

## Context

The CM740 controller communicates with MX28 servos via RS485 half-duplex. Individual servo velocity reads (`PRESENT_SPEED_L` register) return 0 even when servos physically move, likely due to RS485 timing issues. The bulk read mechanism works for internal sensors but not for individual servo velocity.

This design implements velocity computation from position changes (differential velocity) instead of reading the servo velocity register.

## Approach

Compute velocity from position differences using:
```
velocity = (new_position - prev_position) / delta_time
```

### Constraints
- Velocity computed only when position read succeeds (not every timer tick)
- Track actual timestamp per joint for accurate delta time
- Store previous position and timestamp in JointManager
- Output velocity in degrees/second to match position output in degrees

## Message Interface

No changes needed - `Joint.msg` already has the required fields:

```
uint8 id
float32 position
float32 velocity
```

Output topic `/joint/current_joints` uses `CurrentJoints.msg` which contains `Joint[] joints`.

## Architecture

```
Timer (8ms)
    │
    ▼
tachimawari_node.cpp: control_manager->send_bulk_read_packet()
    │
    ▼
joint_node.cpp: publish_current_joints()
    │
    ▼
joint_manager.cpp: get_current_joints()
    │
    ▼
update_current_joints_from_control_manager()
    │
    ├── For each joint:
    │     read_packet(PRESENT_POSITION_L) → value
    │     if (value != -1):
    │         compute_velocity_from_differential(id, value, now)
    │         update_current_joints()
    │
    ▼
Return current_joints with computed velocity
```

## Component Changes

### joint_manager.hpp

Add member variable and method declaration (using standard types, not rclcpp):

```cpp
#include <chrono>
#include <unordered_map>
#include <utility>

// Member variable - stores previous raw position value and timestamp
std::unordered_map<uint8_t, std::pair<int, std::chrono::steady_clock::time_point>> joint_read_state;

// Method to compute velocity from position differential
void compute_velocity_from_differential(uint8_t id, int new_position,
    const std::chrono::steady_clock::time_point & now);
```

### joint_manager.cpp

Replace the PRESENT_SPEED_L reading with differential computation:

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
      double degrees_per_tick = 360.0 / 4096.0;
      double velocity = (delta_pos * degrees_per_tick) / delta_sec;
      // Find joint and set velocity
      for (auto & joint : current_joints) {
        if (joint.get_id() == id) {
          joint.set_velocity(static_cast<float>(velocity));
          break;
        }
      }
    }
  }
  joint_read_state[id] = {new_position, now};
}
```

Modify `update_current_joints_from_control_manager()`:
- Remove `PRESENT_SPEED_L` read entirely
- Call `compute_velocity_from_differential()` when position read succeeds
- Pass timestamp from joint_node (via new parameter or thread-safe clock)

### joint_node.cpp

Two options for timestamp handling:

**Option A:** Pass timestamp to JointManager method
- Add `std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();`
- Pass to `get_current_joints(now)` or add a separate `update_velocity(now)` method

**Option B:** Have JointManager use its own clock
- Add `std::chrono::steady_clock::time_point last_update_` member
- Compute delta internally on each call

Option A is recommended for testability - JointManager can accept mock timestamps.

## Error Handling

| Scenario | Handling |
|----------|----------|
| `read_packet` returns -1 | Skip velocity computation, keep previous velocity |
| First read (no previous) | Initialize state without computing velocity |
| `delta_sec` is 0 | Skip computation (avoid division by zero) |
| Position read succeeds | Compute and set velocity, update state |

## Conversion Details

MX28 has 4096 ticks per rotation:
- 360 degrees = 4096 ticks
- `degrees_per_tick = 360.0 / 4096.0 ≈ 0.0879`

Velocity formula:
```
delta_ticks = new_position - prev_position
delta_sec = now - prev_time
velocity_deg_per_sec = (delta_ticks * 360.0 / 4096.0) / delta_sec
```

## Testing

1. **Unit test**: Mock ControlManager, verify velocity computation with known position deltas
2. **Integration test**: Verify `/joint/current_joints` publishes non-zero velocity when servos physically move
3. **Edge case test**: Verify velocity goes to 0 when servo is stationary
