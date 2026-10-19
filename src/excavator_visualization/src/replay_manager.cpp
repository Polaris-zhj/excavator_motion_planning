#include "excavator_visualization/replay_manager.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

void WriteValues(std::ostream& output, const JointPositions& values,
                 std::size_t joint_count) {
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    const double value = joint < values.size() ? values[joint] : 0.0;
    output << ',' << value;
  }
}

bool ParseNumber(const std::string& text, double* value) {
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool WriteFile(const std::string& path, const std::string& content,
               std::string* error) {
  std::ofstream file(path, std::ios::trunc);
  if (!file) {
    SetError(error, "failed to open replay file for writing: " + path);
    return false;
  }
  file << content;
  if (!file) {
    SetError(error, "failed to write replay file: " + path);
    return false;
  }
  return true;
}

std::string BuildContent(const std::vector<std::string>& joint_names,
                         const std::vector<JointTrajectoryPoint>& points) {
  std::ostringstream output;
  output << std::setprecision(17);
  output << "joint_names";
  for (const std::string& name : joint_names) {
    output << ',' << name;
  }
  output << '\n';
  for (const JointTrajectoryPoint& point : points) {
    output << point.t;
    WriteValues(output, point.positions, joint_names.size());
    WriteValues(output, point.velocities, joint_names.size());
    WriteValues(output, point.accelerations, joint_names.size());
    output << '\n';
  }
  return output.str();
}

bool ReadContent(const std::string& path, std::vector<std::string>* joint_names,
                 std::vector<JointTrajectoryPoint>* points,
                 std::string* error) {
  std::ifstream file(path);
  if (!file) {
    SetError(error, "failed to open replay file: " + path);
    return false;
  }
  std::string line;
  if (!std::getline(file, line)) {
    SetError(error, "replay file is empty: " + path);
    return false;
  }
  const std::vector<std::string> header = Split(line);
  if (header.size() < 2 || header.front() != "joint_names") {
    SetError(error, "replay file must start with a joint_names header");
    return false;
  }
  joint_names->assign(header.begin() + 1, header.end());
  for (const std::string& name : *joint_names) {
    if (name.empty()) {
      SetError(error, "replay file contains an empty joint name");
      return false;
    }
  }
  const std::size_t joint_count = joint_names->size();
  const std::size_t expected_columns = 1 + 3 * joint_count;
  points->clear();
  std::size_t line_number = 1;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = Split(line);
    if (fields.size() != expected_columns) {
      SetError(error, "replay file line " + std::to_string(line_number) +
                          " has the wrong number of columns");
      return false;
    }
    JointTrajectoryPoint point;
    if (!ParseNumber(fields[0], &point.t)) {
      SetError(error, "replay file line " + std::to_string(line_number) +
                          " has an invalid time");
      return false;
    }
    point.positions.resize(joint_count);
    point.velocities.resize(joint_count);
    point.accelerations.resize(joint_count);
    JointPositions* groups[3] = {&point.positions, &point.velocities,
                                 &point.accelerations};
    for (std::size_t group = 0; group < 3; ++group) {
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const std::size_t column = 1 + group * joint_count + joint;
        if (!ParseNumber(fields[column], &(*groups[group])[joint])) {
          SetError(error, "replay file line " + std::to_string(line_number) +
                              " contains an invalid value");
          return false;
        }
      }
    }
    points->push_back(std::move(point));
  }
  if (points->empty()) {
    SetError(error, "replay file contains no data rows: " + path);
    return false;
  }
  return true;
}

}  // namespace

bool ReplayManager::SaveTrajectory(const std::string& path,
                                   const JointTrajectory& trajectory,
                                   std::string* error) const {
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    SetError(error, "trajectory must not be empty");
    return false;
  }
  return WriteFile(
      path, BuildContent(trajectory.joint_names, trajectory.points), error);
}

bool ReplayManager::LoadTrajectory(const std::string& path,
                                   JointTrajectory* trajectory,
                                   std::string* error) const {
  if (trajectory == nullptr) {
    SetError(error, "trajectory output pointer is null");
    return false;
  }
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
  if (!ReadContent(path, &joint_names, &points, error)) {
    return false;
  }
  trajectory->joint_names = std::move(joint_names);
  trajectory->points = std::move(points);
  return true;
}

bool ReplayManager::SaveStates(const std::string& path,
                               const std::vector<std::string>& joint_names,
                               const std::vector<RobotState>& states,
                               std::string* error) const {
  if (joint_names.empty() || states.empty()) {
    SetError(error, "state log must contain joint names and samples");
    return false;
  }
  std::vector<JointTrajectoryPoint> points;
  points.reserve(states.size());
  for (const RobotState& state : states) {
    if (state.positions.size() != joint_names.size()) {
      SetError(error, "state positions must match the joint count");
      return false;
    }
    JointTrajectoryPoint point;
    point.t = state.t;
    point.positions = state.positions;
    point.velocities = state.velocities;
    point.accelerations = state.accelerations;
    points.push_back(std::move(point));
  }
  return WriteFile(path, BuildContent(joint_names, points), error);
}

bool ReplayManager::LoadStates(const std::string& path,
                               std::vector<std::string>* joint_names,
                               std::vector<RobotState>* states,
                               std::string* error) const {
  if (joint_names == nullptr || states == nullptr) {
    SetError(error, "state log output pointers are null");
    return false;
  }
  std::vector<JointTrajectoryPoint> points;
  if (!ReadContent(path, joint_names, &points, error)) {
    return false;
  }
  states->clear();
  states->reserve(points.size());
  for (const JointTrajectoryPoint& point : points) {
    RobotState state;
    state.t = point.t;
    state.positions = point.positions;
    state.velocities = point.velocities;
    state.accelerations = point.accelerations;
    states->push_back(std::move(state));
  }
  return true;
}

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
