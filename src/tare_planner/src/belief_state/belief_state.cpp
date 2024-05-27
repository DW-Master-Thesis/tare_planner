#include "../../include/belief_state/belief_state.h"
#include <cmath>

namespace belief_state_ns {

double Position::distance(const Position& other) const {
  return std::sqrt(std::pow(x_ - other.x(), 2) + std::pow(y_ - other.y(), 2));
}

void Path::append(const Position& position) {
  positions_.push_back(position);
}

void Path::extend(const Path& path) {
  for (const auto& position : path.positions_) {
    positions_.push_back(position);
  }
}

double Path::distance(const Position& position) const {
  double distance = 0.0;
  if (positions_.size() == 0) {
    return 0.0;
  }
  if (positions_.size() == 1) {
    return positions_[0].distance(position);
  }

  auto is_p3_between_p1_and_p2 = [](const Position& p1, const Position& p2, const Position& p3) -> bool {
    return (p1.x() < p3.x() && p3.x() < p2.x()) || (p1.x() > p3.x() && p3.x() > p2.x()) &&
           (p1.y() < p3.y() && p3.y() < p2.y()) || (p1.y() > p3.y() && p3.y() > p2.y());
  };
  auto calc_p3_prime = [](const Position& p1, const Position& p2, const Position& p3) -> Position {
    double m = (p2.y() - p1.y()) / (p2.x() - p1.x());
    double b = p1.y() - m * p1.x();
    double y3_ = m * p3.x() + b;
    return Position(p3.x(), y3_);
  };

  std::vector<double> distances;
  for (size_t i = 0; i < positions_.size() - 1; ++i) {
    const Position& p1 = positions_[i];
    const Position& p2 = positions_[i + 1];
    Position p3 = position;
    Position p3_;
    if (p2.x() - p1.x() != 0) {
      p3_ = calc_p3_prime(p1, p2, p3);
    } else {
      p3_ = Position(p1.x(), p3.y());
    }
    if (is_p3_between_p1_and_p2(p1, p2, p3_)) {
      distances.push_back(p3_.distance(p3));
    } else {
      distances.push_back(std::min(p1.distance(p3), p2.distance(p3)));
    }
  }
  double min_distance = distances[0];
  for (const auto& distance : distances) {
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double BeliefState::calc_cost(const Position& target_position) const {
  double distance;
  double dist_to_robot = robot_position_.distance(target_position);
  double dist_to_plan = robot_plan_.distance(target_position);
  double min_dist = std::min(dist_to_robot, dist_to_plan);
  if (min_dist > limit_) {
    return 0.0;
  }
  double cost = k_1_ * std::exp(-k_2_ * min_dist);
  if (cost > 1.0) {
    return 1.0;
  }
  return cost;
}

double AggregatedBeliefState::calc_cost(const Position& target_position) const {
  std::vector<double> costs;
  for (const auto& belief_state : belief_states_) {
    costs.push_back(belief_state.calc_cost(target_position));
  }
  double max_cost = costs[0];
  for (const auto& cost : costs) {
    if (cost > max_cost) {
      max_cost = cost;
    }
  }
  return max_cost;
}
}
