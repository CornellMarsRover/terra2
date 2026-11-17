// C++ port of cmr_servo_control/end_effector_servo_mapper.py.
#include "cmr_servo_control_cpp/end_effector_servo_mapper.hpp"

#include <algorithm>

namespace cmr_servo_control_cpp
{

const char * const SERVO_EE0 = "ee0";
const char * const SERVO_EE11 = "ee11";
const char * const SERVO_EE15 = "ee15";

const std::vector<std::pair<std::string, int>> & initial_state()
{
  static const std::vector<std::pair<std::string, int>> v = {
    {SERVO_EE0, DRILL_STOP_DEGREES},
    {SERVO_EE11, 100},
    {SERVO_EE15, 95},
  };
  return v;
}

const std::vector<std::pair<std::string, int>> & servo_ids()
{
  static const std::vector<std::pair<std::string, int>> v = {
    {SERVO_EE0, 0},
    {SERVO_EE11, 11},
    {SERVO_EE15, 15},
  };
  return v;
}

const std::map<std::string, int> & servo_can_ids()
{
  static const std::map<std::string, int> m = {
    {SERVO_EE0, 25},
    {SERVO_EE11, 25},
    {SERVO_EE15, 26},
  };
  return m;
}

const std::map<std::string, int> & servo_min()
{
  static const std::map<std::string, int> m = {
    {SERVO_EE11, 40},
    {SERVO_EE15, 75},
  };
  return m;
}

const std::map<std::string, int> & servo_max()
{
  static const std::map<std::string, int> m = {
    {SERVO_EE11, 150},
    {SERVO_EE15, 120},
  };
  return m;
}

int clamp(int value, int minimum, int maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

EndEffectorServoMapper::EndEffectorServoMapper()
: previous_dpad_(DPAD_NEUTRAL)
{
  for (const auto & kv : initial_state()) {
    state_[kv.first] = kv.second;
  }
}

std::vector<EndEffectorServoMapper::Command> EndEffectorServoMapper::update(
  const std::vector<int32_t> & button_array, int dpad)
{
  std::vector<std::optional<Command>> commands;

  if (dpad != previous_dpad_) {
    if (dpad == DPAD_UP) {
      commands.push_back(step_servo(SERVO_EE11, STEP_DEGREES));
    } else if (dpad == DPAD_DOWN) {
      commands.push_back(step_servo(SERVO_EE11, -STEP_DEGREES));
    } else if (dpad == DPAD_LEFT) {
      commands.push_back(toggle_servo(SERVO_EE0, DRILL_REVERSE_DEGREES, DRILL_STOP_DEGREES));
    } else if (dpad == DPAD_RIGHT) {
      commands.push_back(toggle_servo(SERVO_EE0, DRILL_FORWARD_DEGREES, DRILL_STOP_DEGREES));
    }
  }

  if (rising_edge(button_array, BUTTON_TRIANGLE_INDEX)) {
    commands.push_back(step_servo(SERVO_EE15, STEP_DEGREES));
  }
  if (rising_edge(button_array, BUTTON_X_INDEX)) {
    commands.push_back(step_servo(SERVO_EE15, -STEP_DEGREES));
  }

  previous_dpad_ = dpad;
  previous_buttons_ = button_array;

  std::vector<Command> out;
  for (auto & c : commands) {
    if (c) {out.push_back(*c);}
  }
  return out;
}

std::optional<EndEffectorServoMapper::Command> EndEffectorServoMapper::step_servo(
  const std::string & name, int delta)
{
  return set_servo(name, state_.at(name) + delta);
}

std::optional<EndEffectorServoMapper::Command> EndEffectorServoMapper::toggle_servo(
  const std::string & name, int active_angle, int inactive_angle)
{
  if (state_.at(name) == active_angle) {
    return set_servo(name, inactive_angle);
  }
  return set_servo(name, active_angle);
}

std::optional<EndEffectorServoMapper::Command> EndEffectorServoMapper::set_servo(
  const std::string & name, int angle)
{
  const auto mn = servo_min().find(name);
  if (mn != servo_min().end()) {
    angle = clamp(angle, mn->second, servo_max().at(name));
  }
  if (state_.at(name) == angle) {
    return std::nullopt;
  }
  state_[name] = angle;
  return Command{name, angle};
}

bool EndEffectorServoMapper::rising_edge(
  const std::vector<int32_t> & button_array, int index) const
{
  const bool current = button_pressed(button_array, index);
  const bool previous = button_pressed(previous_buttons_, index);
  return current && !previous;
}

bool EndEffectorServoMapper::button_pressed(
  const std::vector<int32_t> & button_array, int index)
{
  return index < static_cast<int>(button_array.size()) && button_array[index] != 0;
}

}  // namespace cmr_servo_control_cpp
