/**
 * @file viewpoint.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.1
 * @date 2019-11-04
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "viewpoint/viewpoint.h"

namespace viewpoint_ns
{
ViewPoint::ViewPoint(double x, double y, double z)
  : lidar_model_(x, y, z)
  , in_collision_(false)
  , in_line_of_sight_(false)
  , connected_(false)
  , visited_(false)
  , selected_(false)
  , is_candidate_(false)
  , has_terrain_height_(false)
  , has_terrain_neighbor_(false)
  , in_exploring_cell_(false)
  , cell_ind_(-1)
  , collision_frame_count_(0)
  , terrain_height_(0.0)
{
}

ViewPoint::ViewPoint(const geometry_msgs::msg::Point& position) : ViewPoint(position.x, position.y, position.z)
{
}

tare_planner_interfaces::msg::Viewpoint ViewPoint::ToMsg() const
{
  tare_planner_interfaces::msg::Viewpoint msg;
  msg.lidar_model = lidar_model_.ToMsg();
  msg.in_collision = in_collision_;
  msg.in_line_of_sight = in_line_of_sight_;
  msg.connected = connected_;
  msg.visited = visited_;
  msg.selected = selected_;
  msg.is_candidate = is_candidate_;
  msg.has_terrain_height = has_terrain_height_;
  msg.in_exploring_cell = in_exploring_cell_;
  msg.cell_ind = cell_ind_;
  msg.collision_frame_count = collision_frame_count_;
  msg.terrain_height = terrain_height_;
  msg.has_terrain_neighbor = has_terrain_neighbor_;
  msg.in_current_frame_line_of_sight = in_current_frame_line_of_sight_;
  msg.covered_point_list = covered_point_list_;
  msg.covered_frontier_point_list = covered_frontier_point_list_;
  return msg;
}

void ViewPoint::FromMsg(const tare_planner_interfaces::msg::Viewpoint& msg)
{
  covered_point_list_.clear();
  covered_frontier_point_list_.clear();
  lidar_model_.FromMsg(msg.lidar_model);
  in_collision_ = msg.in_collision;
  in_line_of_sight_ = msg.in_line_of_sight;
  connected_ = msg.connected;
  visited_ = msg.visited;
  selected_ = msg.selected;
  is_candidate_ = msg.is_candidate;
  has_terrain_height_ = msg.has_terrain_height;
  in_exploring_cell_ = msg.in_exploring_cell;
  cell_ind_ = msg.cell_ind;
  collision_frame_count_ = msg.collision_frame_count;
  terrain_height_ = msg.terrain_height;
  has_terrain_neighbor_ = msg.has_terrain_neighbor;
  in_current_frame_line_of_sight_ = msg.in_current_frame_line_of_sight;
  covered_point_list_ = msg.covered_point_list;
  covered_frontier_point_list_ = msg.covered_frontier_point_list;
}

void ViewPoint::Merge(const ViewPoint& other)
{
  lidar_model_ = other.lidar_model_;
  in_collision_ = in_collision_ || other.in_collision_;
  connected_ = connected_ || other.connected_;
  visited_ = visited_ || other.visited_;
  selected_ = selected_ || other.selected_;
  is_candidate_ = is_candidate_ || other.is_candidate_;
}

void ViewPoint::Reset()
{
  in_collision_ = false;
  in_line_of_sight_ = false;
  connected_ = false;
  visited_ = false;
  selected_ = false;
  is_candidate_ = false;
  has_terrain_height_ = false;
  has_terrain_neighbor_ = false;
  in_exploring_cell_ = false;
  cell_ind_ = -1;
  lidar_model_.ResetCoverage();
  covered_point_list_.clear();
  covered_frontier_point_list_.clear();
  collision_frame_count_ = 0;
  terrain_height_ = 0.0;
}

void ViewPoint::ResetCoverage()
{
  lidar_model_.ResetCoverage();
  covered_point_list_.clear();
  covered_frontier_point_list_.clear();
}
}  // namespace viewpoint_ns
