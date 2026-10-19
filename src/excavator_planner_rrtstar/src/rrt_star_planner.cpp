#include "excavator_planner_rrtstar/rrt_star_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace tramcar {
namespace planning {
namespace {

constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
constexpr double kEpsilon = 1.0e-9;

bool IsFinite(const JointPositions& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

double Distance(const JointPositions& first, const JointPositions& second) {
  double squared = 0.0;
  for (std::size_t i = 0; i < first.size(); ++i) {
    const double delta = first[i] - second[i];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

JointPositions Steer(const JointPositions& from, const JointPositions& to,
                     double step_size) {
  const double distance = Distance(from, to);
  if (distance <= step_size || distance < kEpsilon) {
    return to;
  }
  const double ratio = step_size / distance;
  JointPositions result(from.size(), 0.0);
  for (std::size_t i = 0; i < from.size(); ++i) {
    result[i] = from[i] + ratio * (to[i] - from[i]);
  }
  return result;
}

JointPositions Interpolate(const JointPositions& start,
                           const JointPositions& goal, double ratio) {
  JointPositions result(start.size(), 0.0);
  for (std::size_t i = 0; i < start.size(); ++i) {
    result[i] = start[i] + ratio * (goal[i] - start[i]);
  }
  return result;
}

}  // namespace

void RrtStarPlanner::Configure(const PlannerSettings& settings) {
  settings_ = settings;
  configured_ = settings_.joint_names.size() > 0 &&
                settings_.joint_limits.size() ==
                    settings_.joint_names.size() &&
                std::isfinite(settings_.max_joint_step) &&
                settings_.max_joint_step > 0.0 &&
                std::isfinite(settings_.step_size) &&
                settings_.step_size > 0.0 && settings_.max_iterations > 0 &&
                std::isfinite(settings_.goal_bias) &&
                settings_.goal_bias >= 0.0 && settings_.goal_bias <= 1.0 &&
                std::isfinite(settings_.rewire_radius) &&
                settings_.rewire_radius > 0.0 &&
                std::isfinite(settings_.goal_tolerance) &&
                settings_.goal_tolerance >= 0.0 &&
                std::isfinite(settings_.collision_check_resolution) &&
                settings_.collision_check_resolution > 0.0;
}

PlanningResponse RrtStarPlanner::Plan(const PlanningRequest& request) {
  using Clock = std::chrono::steady_clock;
  PlanningResponse response;
  if (!configured_) {
    response.message = "RRT* planner is not configured";
    return response;
  }
  const std::size_t joint_count = settings_.joint_names.size();
  if (request.start.size() != joint_count ||
      request.goal.size() != joint_count || !IsFinite(request.start) ||
      !IsFinite(request.goal)) {
    response.message = "RRT* start and goal must match the joint count";
    return response;
  }
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    if (settings_.joint_limits[joint].first >
        settings_.joint_limits[joint].second) {
      response.message = "RRT* joint limit lower exceeds upper";
      return response;
    }
    if (request.start[joint] < settings_.joint_limits[joint].first ||
        request.start[joint] > settings_.joint_limits[joint].second ||
        request.goal[joint] < settings_.joint_limits[joint].first ||
        request.goal[joint] > settings_.joint_limits[joint].second) {
      response.message = "RRT* start or goal violates a joint limit";
      return response;
    }
  }

  double validity_seconds = 0.0;
  std::size_t validity_checks = 0;
  const auto state_valid = [&request, &validity_seconds,
                            &validity_checks](const JointPositions& positions) {
    if (!request.state_validity_check) {
      return true;
    }
    const auto start_time = Clock::now();
    std::string message;
    const bool valid = request.state_validity_check(positions, &message);
    validity_seconds +=
        std::chrono::duration<double>(Clock::now() - start_time).count();
    ++validity_checks;
    return valid;
  };

  const double edge_resolution =
      std::max(1.0e-4, settings_.collision_check_resolution);
  const auto segment_valid = [&state_valid, edge_resolution](
                                 const JointPositions& start,
                                 const JointPositions& goal) {
    const double distance = Distance(start, goal);
    const std::size_t steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(distance / edge_resolution)));
    for (std::size_t step = 1; step <= steps; ++step) {
      const JointPositions sample =
          Interpolate(start, goal, static_cast<double>(step) /
                                      static_cast<double>(steps));
      if (!state_valid(sample)) {
        return false;
      }
    }
    return true;
  };

  const double budget = std::isfinite(request.timeout_seconds) &&
                                request.timeout_seconds > 0.0
                            ? request.timeout_seconds
                            : 1.0;
  const Clock::time_point deadline =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(budget));
  const auto should_continue = [&request, &deadline]() {
    if (request.should_continue && !request.should_continue()) {
      return false;
    }
    return Clock::now() < deadline;
  };

  if (!state_valid(request.start)) {
    response.message = "RRT* start state is invalid";
    return response;
  }
  if (!state_valid(request.goal)) {
    response.message = "RRT* goal state is invalid";
    return response;
  }

  std::mt19937 rng(settings_.random_seed);
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  std::vector<std::pair<double, double>> sample_bounds;
  sample_bounds.reserve(joint_count);
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    double lower = settings_.joint_limits[joint].first;
    double upper = settings_.joint_limits[joint].second;
    if (!std::isfinite(lower)) {
      lower = std::min(request.start[joint], request.goal[joint]) - 1.0;
    }
    if (!std::isfinite(upper)) {
      upper = std::max(request.start[joint], request.goal[joint]) + 1.0;
    }
    if (!(lower < upper)) {
      response.message = "RRT* sampling bounds are degenerate";
      return response;
    }
    sample_bounds.emplace_back(lower, upper);
  }

  std::vector<Node> nodes;
  nodes.push_back({request.start, 0, 0.0});
  const double connect_radius =
      std::max(settings_.goal_tolerance, settings_.step_size);

  auto tree_path_cost = [&nodes](std::size_t index) {
    double cost = 0.0;
    std::size_t current = index;
    std::size_t guard = 0;
    while (current != 0 && guard++ <= nodes.size()) {
      const std::size_t parent = nodes[current].parent;
      if (parent >= nodes.size() || parent == current) {
        return std::numeric_limits<double>::infinity();
      }
      cost += Distance(nodes[current].positions, nodes[parent].positions);
      current = parent;
    }
    return cost;
  };
  auto is_ancestor = [&nodes](std::size_t candidate, std::size_t node) {
    std::size_t current = node;
    std::size_t guard = 0;
    while (guard++ <= nodes.size()) {
      if (current == candidate) {
        return true;
      }
      if (current == 0) {
        break;
      }
      current = nodes[current].parent;
    }
    return false;
  };

  double best_goal_cost = std::numeric_limits<double>::infinity();
  std::size_t best_goal_parent = kNoParent;
  const auto try_connect_goal = [&](std::size_t index) {
    const double distance =
        Distance(nodes[index].positions, request.goal);
    if (distance > connect_radius) {
      return;
    }
    const double cost = tree_path_cost(index) + distance;
    if (cost + kEpsilon >= best_goal_cost) {
      return;
    }
    if (!segment_valid(nodes[index].positions, request.goal)) {
      return;
    }
    best_goal_cost = cost;
    best_goal_parent = index;
  };
  try_connect_goal(0);

  std::size_t completed_iterations = 0;
  bool budget_exhausted = false;
  for (std::size_t iteration = 0; iteration < settings_.max_iterations;
       ++iteration) {
    if (!should_continue()) {
      budget_exhausted = true;
      break;
    }
    JointPositions sample(joint_count, 0.0);
    if (probability(rng) < settings_.goal_bias) {
      sample = request.goal;
    } else {
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        std::uniform_real_distribution<double> distribution(
            sample_bounds[joint].first, sample_bounds[joint].second);
        sample[joint] = distribution(rng);
      }
    }

    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double distance = Distance(nodes[index].positions, sample);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = index;
      }
    }
    const JointPositions new_positions =
        Steer(nodes[nearest].positions, sample, settings_.step_size);
    if (Distance(new_positions, nodes[nearest].positions) < kEpsilon ||
        !state_valid(new_positions) ||
        !segment_valid(nodes[nearest].positions, new_positions)) {
      completed_iterations = iteration + 1;
      continue;
    }

    std::vector<std::pair<std::size_t, double>> near_nodes;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double distance = Distance(nodes[index].positions, new_positions);
      if (distance <= settings_.rewire_radius) {
        near_nodes.emplace_back(index, distance);
      }
    }

    std::size_t best_parent = nearest;
    double best_cost =
        nodes[nearest].cost + Distance(nodes[nearest].positions, new_positions);
    for (const auto& near : near_nodes) {
      if (near.first == nearest) {
        continue;
      }
      const double candidate_cost = nodes[near.first].cost + near.second;
      if (candidate_cost + kEpsilon >= best_cost) {
        continue;
      }
      if (!segment_valid(nodes[near.first].positions, new_positions)) {
        continue;
      }
      best_parent = near.first;
      best_cost = candidate_cost;
    }

    const std::size_t new_index = nodes.size();
    nodes.push_back({new_positions, best_parent, best_cost});

    for (const auto& near : near_nodes) {
      if (near.first == best_parent ||
          is_ancestor(near.first, new_index)) {
        continue;
      }
      const double rewired_cost = best_cost + near.second;
      if (rewired_cost + kEpsilon >= nodes[near.first].cost) {
        continue;
      }
      if (!segment_valid(new_positions, nodes[near.first].positions)) {
        continue;
      }
      nodes[near.first].parent = new_index;
      nodes[near.first].cost = rewired_cost;
      try_connect_goal(near.first);
    }
    try_connect_goal(new_index);
    completed_iterations = iteration + 1;
  }

  if (best_goal_parent == kNoParent) {
    response.message =
        budget_exhausted
            ? "RRT* planning canceled or timed out before finding a path"
            : "RRT* did not find a path within the iteration limit";
    return response;
  }

  std::vector<std::size_t> chain;
  std::size_t current = best_goal_parent;
  std::size_t guard = 0;
  while (guard++ <= nodes.size()) {
    chain.push_back(current);
    if (current == 0) {
      break;
    }
    current = nodes[current].parent;
  }
  if (chain.empty() || chain.back() != 0) {
    response.message = "RRT* failed to reconstruct the path";
    return response;
  }
  std::reverse(chain.begin(), chain.end());

  JointPath path;
  path.joint_names = settings_.joint_names;
  path.points.reserve(chain.size() + 1);
  for (std::size_t index : chain) {
    path.points.push_back(nodes[index].positions);
  }
  // Always include an explicit goal waypoint (even for a zero-distance goal)
  // so downstream time parameterization always receives at least two points.
  path.points.push_back(request.goal);
  const ValidationResult validation = ValidateJointPath(path);
  if (!validation.ok) {
    response.message = "RRT* produced an invalid path: " + validation.message;
    return response;
  }

  response.success = true;
  response.path = std::move(path);
  const double average_check_ms =
      validity_checks > 0 ? 1000.0 * validity_seconds /
                                static_cast<double>(validity_checks)
                          : 0.0;
  response.message = "RRT* path found: nodes=" +
                     std::to_string(nodes.size()) + ", iterations=" +
                     std::to_string(completed_iterations) + ", cost=" +
                     std::to_string(best_goal_cost) + ", validity_checks=" +
                     std::to_string(validity_checks) + ", avg_check_ms=" +
                     std::to_string(average_check_ms);
  return response;
}

}  // namespace planning
}  // namespace tramcar

PLUGINLIB_EXPORT_CLASS(tramcar::planning::RrtStarPlanner,
                       tramcar::planning::PlannerPlugin)
