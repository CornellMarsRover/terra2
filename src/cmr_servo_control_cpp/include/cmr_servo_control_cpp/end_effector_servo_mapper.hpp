// C++ port of cmr_servo_control/end_effector_servo_mapper.py.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cmr_servo_control_cpp
{

constexpr int DPAD_UP = 0;
constexpr int DPAD_RIGHT = 2;
constexpr int DPAD_DOWN = 4;
constexpr int DPAD_LEFT = 6;
constexpr int DPAD_NEUTRAL = 8;

constexpr int BUTTON_X_INDEX = 5;
constexpr int BUTTON_TRIANGLE_INDEX = 7;

constexpr int STEP_DEGREES = 5;
constexpr int DRILL_STOP_DEGREES = 90;
constexpr int DRILL_REVERSE_DEGREES = 0;
constexpr int DRILL_FORWARD_DEGREES = 180;

extern const char * const SERVO_EE0;
extern const char * const SERVO_EE11;
extern const char * const SERVO_EE15;

// Ordered like the Python dict literals (ee0, ee11, ee15).
const std::vector<std::pair<std::string, int>> & initial_state();
const std::vector<std::pair<std::string, int>> & servo_ids();
const std::map<std::string, int> & servo_can_ids();
const std::map<std::string, int> & servo_min();
const std::map<std::string, int> & servo_max();

int clamp(int value, int minimum, int maximum);

class EndEffectorServoMapper
{
public:
  using Command = std::pair<std::string, int>;

  EndEffectorServoMapper();

  std::vector<Command> update(const std::vector<int32_t> & button_array, int dpad);

  const std::map<std::string, int> & state() const {return state_;}

private:
  std::optional<Command> step_servo(const std::string & name, int delta);
  std::optional<Command> toggle_servo(
    const std::string & name, int active_angle, int inactive_angle);
  std::optional<Command> set_servo(const std::string & name, int angle);
  bool rising_edge(const std::vector<int32_t> & button_array, int index) const;
  static bool button_pressed(const std::vector<int32_t> & button_array, int index);

  std::map<std::string, int> state_;
  int previous_dpad_;
  std::vector<int32_t> previous_buttons_;
};

}  // namespace cmr_servo_control_cpp
