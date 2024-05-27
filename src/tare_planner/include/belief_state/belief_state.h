#pragma once

#include <vector>

namespace belief_state_ns {

class Position {
public:
  Position() : x_(0.0), y_(0.0) {}
  Position(double x, double y) : x_(x), y_(y) {}
  double x() const { return x_; }
  double y() const { return y_; }
  double distance(const Position& other) const;

private:
  double x_;
  double y_;
};


class Path {
public:
  Path() {}
  void append(const Position& position);
  void extend(const Path& path);
  double distance(const Position& position) const;

private:
  std::vector<Position> positions_;
};


class BeliefState {
public:
  BeliefState(
    Position robot_position,
    Path robot_plan,
    double limit
  ) : robot_position_(robot_position), robot_plan_(robot_plan), limit_(limit) {};
  double calc_cost(const Position& target_position) const;

private:
  double k_1_ = 0.8;
  double k_2_ = 0.02;
  Position robot_position_;
  Path robot_plan_;
  double limit_;
};


class AggregatedBeliefState {
public:
  AggregatedBeliefState(
    std::vector<BeliefState> belief_states
  ) : belief_states_(belief_states) {};
  double calc_cost(const Position& target_position) const;

private:
  std::vector<BeliefState> belief_states_;
};

}
