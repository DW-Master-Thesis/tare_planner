/**
 * @file grid_world.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../../include/grid_world/grid_world.h"
#include <queue>
#include <algorithm>
#include <utils/misc_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <global_plan_interfaces/msg/cell.hpp>
#include <global_plan_interfaces/msg/robot.hpp>
#include <global_plan_interfaces/msg/connection_between_nodes.hpp>

namespace grid_world_ns
{
Cell::Cell(double x, double y, double z)
  : in_horizon_(false)
  , robot_position_set_(false)
  , visit_count_(0)
  , keypose_id_(0)
  , path_added_to_keypose_graph_(false)
  , roadmap_connection_point_set_(false)
  , viewpoint_position_(Eigen::Vector3d(x, y, z))
  , roadmap_connection_point_(Eigen::Vector3d(x, y, z))
{
  center_.x = x;
  center_.y = y;
  center_.z = z;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  status_ = CellStatus::UNSEEN;
  exploring_status_ = ExploringStatus::NotInGlobalPlan;
}

Cell::Cell(const geometry_msgs::msg::Point& center) : Cell(center.x, center.y, center.z)
{
}

void Cell::Reset()
{
  status_ = CellStatus::UNSEEN;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  visit_count_ = 0;
  viewpoint_indices_.clear();
  connected_cell_indices_.clear();
  keypose_graph_node_indices_.clear();
}

bool Cell::IsCellConnected(int cell_ind)
{
  if (std::find(connected_cell_indices_.begin(), connected_cell_indices_.end(), cell_ind) !=
      connected_cell_indices_.end())
  {
    return true;
  }
  else
  {
    return false;
  }
}

GridWorld::GridWorld(rclcpp::Node::SharedPtr nh) : initialized_(false), use_keypose_graph_(false), exploring_status_updated_(false)
{
  ReadParameters(nh);
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;

  cur_robot_cell_ind_ = -1;
  prev_robot_cell_ind_ = -1;
}

GridWorld::GridWorld(int row_num, int col_num, int level_num, double cell_size, double cell_height, int nearby_grid_num)
  : kRowNum(row_num)
  , kColNum(col_num)
  , kLevelNum(level_num)
  , kCellSize(cell_size)
  , kCellHeight(cell_height)
  , KNearbyGridNum(nearby_grid_num)
  , kMinAddPointNumSmall(60)
  , kMinAddPointNumBig(100)
  , kMinAddFrontierPointNum(30)
  , kCellExploringToCoveredThr(1)
  , kCellCoveredToExploringThr(10)
  , kCellExploringToAlmostCoveredThr(10)
  , kCellAlmostCoveredToExploringThr(20)
  , kCellUnknownToExploringThr(1)
  , cur_keypose_id_(0)
  , cur_keypose_graph_node_ind_(0)
  , cur_robot_cell_ind_(-1)
  , prev_robot_cell_ind_(-1)
  , cur_keypose_(0, 0, 0)
  , initialized_(false)
  , use_keypose_graph_(false)
{
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;
}

void GridWorld::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kGridWorldXNum", kRowNum);
  nh->get_parameter("kGridWorldYNum", kColNum);
  nh->get_parameter("kGridWorldZNum", kLevelNum);
  int viewpoint_number = nh->get_parameter("viewpoint_manager/number_x").as_int();
  double viewpoint_resolution = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  kCellSize = viewpoint_number * viewpoint_resolution / 5;
  nh->get_parameter("kGridWorldCellHeight", kCellHeight);
  nh->get_parameter("kGridWorldNearbyGridNum", KNearbyGridNum);
  nh->get_parameter("kMinAddPointNumSmall", kMinAddPointNumSmall);
  nh->get_parameter("kMinAddPointNumBig", kMinAddPointNumBig);
  nh->get_parameter("kMinAddFrontierPointNum", kMinAddFrontierPointNum);
  nh->get_parameter("kCellExploringToCoveredThr", kCellExploringToCoveredThr);
  nh->get_parameter("kCellCoveredToExploringThr", kCellCoveredToExploringThr);
  nh->get_parameter("kCellExploringToAlmostCoveredThr", kCellExploringToAlmostCoveredThr);
  nh->get_parameter("kCellAlmostCoveredToExploringThr", kCellAlmostCoveredToExploringThr);
  nh->get_parameter("kCellUnknownToExploringThr", kCellUnknownToExploringThr);
}

void GridWorld::UpdateNeighborCells(const geometry_msgs::msg::Point& robot_position)
{
  if (!initialized_)
  {
    initialized_ = true;
    origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
    origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
    origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
    subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
    // Update cell centers
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
          geometry_msgs::msg::Point subspace_center_geo_position;
          subspace_center_geo_position.x = subspace_center_position.x();
          subspace_center_geo_position.y = subspace_center_position.y();
          subspace_center_geo_position.z = subspace_center_position.z();
          subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
          subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
        }
      }
    }
  }

  // Get neighbor cells
  std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_;
  neighbor_cell_indices_.clear();
  int N = KNearbyGridNum / 2;
  int M = 1;
  GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M), neighbor_cell_indices_);

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
        prev_neighbor_cell_indices.end())
    {
      // subspaces_->GetCell(cell_ind).AddVisitCount();
      subspaces_->GetCell(cell_ind).AddVisitCount();
    }
  }
}

void GridWorld::UpdateRobotPosition(const geometry_msgs::msg::Point& robot_position)
{
  robot_position_ = robot_position;
  int robot_cell_ind = GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
  if (cur_robot_cell_ind_ != robot_cell_ind)
  {
    prev_robot_cell_ind_ = cur_robot_cell_ind_;
    cur_robot_cell_ind_ = robot_cell_ind;
  }
}

void GridWorld::UpdateCellKeyposeGraphNodes(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();

  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).ClearGraphNodeIndices();
    subspaces_->GetCell(i).SetRoadmapConnectionPointSet(false);
  }
  for (const auto& node_ind : keypose_graph_connected_node_indices)
  {
    geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);
    int cell_ind = GetCellInd(node_position.x, node_position.y, node_position.z);
    if (subspaces_->InRange(cell_ind))
    {
      subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
    }
  }
  // Determine the connection point in each cell
  for (int cell_ind = 0; cell_ind < subspaces_->GetCellNumber(); cell_ind++)
  {
    // Find candidate viewpoint in current cell that is closest to the cell center
    double min_dist = DBL_MAX;
    double min_dist_viewpoint_ind = -1;
    for (auto j: subspaces_->GetCell(cell_ind).GetGraphNodeIndices())
    {
      geometry_msgs::msg::Point viewpoint_position = keypose_graph->GetNodePosition(j);
      Eigen::Vector3i sub =
          subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
      if (subspaces_->Sub2Ind(sub) != cell_ind)
      {
        continue;
      }
      double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          viewpoint_position, subspaces_->GetCell(cell_ind).GetPosition());
      if (dist_to_cell_center < min_dist)
      {
        min_dist = dist_to_cell_center;
        min_dist_viewpoint_ind = j;
      }
    }
    if (min_dist_viewpoint_ind < 0)
    {
      continue;
    }
    // Set candidate viewpoint as the roadmap connection point
    geometry_msgs::msg::Point min_dist_viewpoint_position =
        keypose_graph->GetNodePosition(min_dist_viewpoint_ind);
    subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(
        Eigen::Vector3d(min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
    subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
  }
  // If not roadmap_connection_point set for current cell, set it to robot position
  if (!subspaces_->GetCell(cur_robot_cell_ind_).IsRoadmapConnectionPointSet())
  {
    int closest_node_ind = keypose_graph->GetClosestNodeInd(robot_position_);
    geometry_msgs::msg::Point closest_node_position = keypose_graph->GetNodePosition(closest_node_ind);
    subspaces_->GetCell(cur_robot_cell_ind_).SetRoadmapConnectionPoint(
        Eigen::Vector3d(closest_node_position.x, closest_node_position.y, closest_node_position.z));
    subspaces_->GetCell(cur_robot_cell_ind_).SetRoadmapConnectionPointSet(true);
  }
}

bool GridWorld::AreNeighbors(int cell_ind1, int cell_ind2)
{
  Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
  Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
  Eigen::Vector3i diff = cell_sub1 - cell_sub2;
  if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int GridWorld::GetCellInd(double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  if (subspaces_->InRange(sub))
  {
    return subspaces_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
  col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
  level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
}

Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point)
{
  return subspaces_->Pos2Sub(point);
}

void GridWorld::GetMarker(visualization_msgs::msg::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;

  int exploring_count = 0;
  int covered_count = 0;
  int unseen_count = 0;

  for (int i = 0; i < kRowNum; i++)
  {
    for (int j = 0; j < kColNum; j++)
    {
      for (int k = 0; k < kLevelNum; k++)
      {
        int cell_ind = subspaces_->Sub2Ind(i, j, k);
        geometry_msgs::msg::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
        std_msgs::msg::ColorRGBA color;
        bool add_marker = true;
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
        {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;
          unseen_count++;
          add_marker = false;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
        {
          color.r = 1.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          covered_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
        {
          color.r = 1.0;
          color.g = 1.0;
          color.b = 0.2;
          color.a = 0.1;
          covered_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
        {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 0.1;
          if (subspaces_->GetCell(cell_ind).GetExploringStatus() == ExploringStatus::InGlobalPlan)
          {
            color.g = 1.0;
          }
          else if (subspaces_->GetCell(cell_ind).GetExploringStatus() == ExploringStatus::InOtherGlobalPlan)
          {
            color.b = 1.0;
          }
          else if (subspaces_->GetCell(cell_ind).GetExploringStatus() == ExploringStatus::NextInGlobalPlan)
          {
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
          }
          else
          {
            color.r = 1.0;
          }
          exploring_count++;
          add_marker = true;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::NOGO)
        {
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 0.1;
        }
        else
        {
          color.r = 0.8;
          color.g = 0.8;
          color.b = 0.8;
          color.a = 0.1;
        }
        if (subspaces_->GetCell(cell_ind).HasRobot())
        {
          color.r = 1.0;
          color.g = 0.4;
          color.b = 1.0;
          color.a = 0.1;
        }
        if (add_marker)
        {
          marker.colors.push_back(color);
          marker.points.push_back(cell_center);
        }
      }
    }
  }
  //        // Color neighbor cells differently
  //        for(const auto & ind : neighbor_cell_indices_){
  //            if(cells_[ind].GetStatus() == CellStatus::UNSEEN) continue;
  //            marker.colors[ind].a = 0.8;
  //        }
}

void GridWorld::GetConnectedCellMarker(visualization_msgs::msg::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;
  
  std::vector<int> visited_cells;
  std::queue<int> queue;
  queue.push(cur_robot_cell_ind_);
  while (!queue.empty())
  {
    int cur_cell_ind = queue.front();
    queue.pop();
    if (std::find(visited_cells.begin(), visited_cells.end(), cur_cell_ind) != visited_cells.end())
    {
      continue;
    }
    visited_cells.push_back(cur_cell_ind);
    geometry_msgs::msg::Point cell_center = subspaces_->GetCell(cur_cell_ind).GetPosition();
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 0.1;
    marker.colors.push_back(color);
    marker.points.push_back(cell_center);
    for (const auto& neighbor_cell_ind : subspaces_->GetCell(cur_cell_ind).GetConnectedCellIndices())
    {
      if (std::find(visited_cells.begin(), visited_cells.end(), neighbor_cell_ind) == visited_cells.end())
      {
        queue.push(neighbor_cell_ind);
      }
    }
  }
}

void GridWorld::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    CellStatus cell_status = subspaces_->GetCell(i).GetStatus();
    if (!subspaces_->GetCell(i).GetConnectedCellIndices().empty())
    {
      pcl::PointXYZI point;
      Eigen::Vector3d position = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = i;
      vis_cloud->points.push_back(point);
    }
  }
}

void GridWorld::AddViewPointToCell(int cell_ind, int viewpoint_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind);
}

void GridWorld::AddGraphNodeToCell(int cell_ind, int node_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
}

void GridWorld::ClearCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ClearViewPointIndices();
}

std::vector<int> GridWorld::GetCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetViewPointIndices();
}

void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  int row_idx = 0;
  int col_idx = 0;
  int level_idx = 0;
  for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)
  {
    for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)
    {
      row_idx = center_cell_sub.x() + i;
      col_idx = center_cell_sub.y() + j;
      for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++)
      {
        level_idx = center_cell_sub.z() + k;
        Eigen::Vector3i sub(row_idx, col_idx, level_idx);
        // if (SubInBound(row_idx, col_idx, level_idx))
        if (subspaces_->InRange(sub))
        {
          // int ind = sub2ind(row_idx, col_idx, level_idx);
          int ind = subspaces_->Sub2Ind(sub);
          neighbor_indices.push_back(ind);
        }
      }
    }
  }
}
void GridWorld::GetNeighborCellIndices(const geometry_msgs::msg::Point& position, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

  GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
}

void GridWorld::GetDistanceAndPathBetweenCells(
  int from_cell_ind,
  int to_cell_ind,
  double& distance,
  nav_msgs::msg::Path& path,
  const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph
)
{
  Eigen::Vector3d from_cell_position = subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
  Eigen::Vector3d to_cell_position = subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
  geometry_msgs::msg::Point from_position;
  from_position.x = from_cell_position.x();
  from_position.y = from_cell_position.y();
  from_position.z = from_cell_position.z();
  geometry_msgs::msg::Point to_position;
  to_position.x = to_cell_position.x();
  to_position.y = to_cell_position.y();
  to_position.z = to_cell_position.z();
  distance = keypose_graph->GetShortestPath(from_position, to_position, true, path, true);
  if (distance <= 0)
  {
    path.poses.clear();
    distance = DBL_MAX;
  }
}

void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices)
{
  exploring_cell_indices.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING && subspaces_->GetCell(i).IsRoadmapConnectionPointSet())
    {
      exploring_cell_indices.push_back(i);
    }
  }
}

CellStatus GridWorld::GetCellStatus(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetStatus();
}

void GridWorld::SetCellStatus(int cell_ind, grid_world_ns::CellStatus status)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetStatus(status);
}

geometry_msgs::msg::Point GridWorld::GetCellPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetPosition();
}

void GridWorld::SetCellRobotPosition(int cell_ind, const geometry_msgs::msg::Point& robot_position)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position);
}

geometry_msgs::msg::Point GridWorld::GetCellRobotPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetRobotPosition();
}

void GridWorld::CellAddVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddVisitCount();
}

int GridWorld::GetCellVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetVisitCount();
}

bool GridWorld::IsRobotPositionSet(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsRobotPositionSet();
}

void GridWorld::Reset()
{
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).Reset();
  }
}

int GridWorld::GetCellStatusCount(grid_world_ns::CellStatus status)
{
  int count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == status)
    {
      count++;
    }
  }
  return count;
}

void GridWorld::UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager, bool others)
{
  std::vector<int> cells_to_update;
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    Eigen::Vector3i sub =
        subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    int cell_ind = subspaces_->Sub2Ind(sub);
    if (std::find(cells_to_update.begin(), cells_to_update.end(), cell_ind) == cells_to_update.end())
    {
      cells_to_update.push_back(cell_ind);
    }
  }

  for (int cell_ind: cells_to_update)
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();
  }
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    Eigen::Vector3i sub =
        subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    int cell_ind = subspaces_->Sub2Ind(sub);
    AddViewPointToCell(cell_ind, viewpoint_ind);
    viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);
  }

  for (int cell_ind: cells_to_update)
  {
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
    {
      continue;
    }
    int candidate_count = 0;
    int selected_viewpoint_count = 0;
    int above_big_threshold_count = 0;
    int above_small_threshold_count = 0;
    int above_frontier_threshold_count = 0;
    bool not_visited = true;
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())
    {
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
      candidate_count++;
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))
      {
        selected_viewpoint_count++;
      }
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        not_visited = false;
        continue;
      }
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
      if (score > kMinAddPointNumSmall)
      {
        above_small_threshold_count++;
      }
      if (score > kMinAddPointNumBig)
      {
        above_big_threshold_count++;
      }
      if (frontier_score > kMinAddFrontierPointNum)
      {
        above_frontier_threshold_count++;
      }
    }
    // Exploring to Covered by others
    if (others &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED_BY_OTHERS);
      // continue;
    }
    // Exploring to Covered
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
    }
    // Covered to Exploring
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
             (above_big_threshold_count >= kCellCoveredToExploringThr ||
              above_frontier_threshold_count >= kCellCoveredToExploringThr))
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // Exploring to Almost covered
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && selected_viewpoint_count == 0 &&
             candidate_count > 0)
    {
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // Any status other than COVERED to Explorting
    else if (subspaces_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED && selected_viewpoint_count > 0 && not_visited)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
    // else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count == 0)
    // {
    //   // First visit
    //   if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
    //       subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
    //   {
    //     subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
    //   }
    //   else
    //   {
    //     geometry_msgs::msg::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
    //     double xy_dist_to_robot = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
    //         cell_position, robot_position_);
    //     double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);
    //     if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
    //     {
    //       subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
    //     }
    //   }
    // }

    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
      subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
    }
  }
  for (const auto& cell_ind : almost_covered_cell_indices_)
  {
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) ==
        neighbor_cell_indices_.end())
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
  }
}

geometry_msgs::msg::Point GridWorld::GetCellCenterFromPosition(const geometry_msgs::msg::Point& position)
{
  int cell_ind = GetCellInd(position.x, position.y, position.z);
  return GetCellPosition(cell_ind);
}

exploration_path_ns::ExplorationPath GridWorld::SolveGlobalVRP(
    const std::vector<geometry_msgs::msg::Point>& other_robot_positions,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph
)
{
  distance_matrix_msg_ = global_plan_interfaces::msg::DistanceMatrix();
  std::vector<global_plan_interfaces::msg::Robot> robots;
  std::vector<global_plan_interfaces::msg::Cell> cells;
  // Reset Explorating status and Has Robot status
  for (int cell_ind = 0; cell_ind < subspaces_->GetCellNumber(); cell_ind++)
  {
    subspaces_->GetCell(cell_ind).SetExploringStatus(ExploringStatus::NotInGlobalPlan);
    subspaces_->GetCell(cell_ind).SetHasRobot(false);
  }

  // Get cell indices of robots
  std::vector<int> all_cell_indices;
  all_cell_indices.push_back(GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z));
  global_plan_interfaces::msg::Robot robot;
  robot.id = 0;
  robot.position = robot_position_;
  robots.push_back(robot);
  distance_matrix_msg_.cell_or_robot_ids.push_back(robot.id);
  distance_matrix_msg_.is_node_robot.push_back(true);
  for (const auto& other_robot_position : other_robot_positions)
  {
    int other_robot_cell_ind = GetCellInd(other_robot_position.x, other_robot_position.y, other_robot_position.z);
    all_cell_indices.push_back(other_robot_cell_ind);
    global_plan_interfaces::msg::Robot other_robot;
    other_robot.id = robots.size();
    other_robot.position = other_robot_position;
    distance_matrix_msg_.robots.push_back(other_robot);
    distance_matrix_msg_.cell_or_robot_ids.push_back(other_robot.id);
    distance_matrix_msg_.is_node_robot.push_back(true);
  }
  // Get exploring cell inidices 
  std::vector<int> exploring_cell_indices;
  std::vector<int> reachable_exploring_cell_indices;
  GetExploringCellIndices(exploring_cell_indices);
  for (int i: exploring_cell_indices)
  {
    global_plan_interfaces::msg::Cell cell;
    cell.id = i;
    cell.position = subspaces_->GetCell(i).GetPosition();
    Eigen::Vector3d connection_point = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
    cell.connection_point.x = connection_point.x();
    cell.connection_point.y = connection_point.y();
    cell.connection_point.z = connection_point.z();
    cells.push_back(cell);
    double distance;
    nav_msgs::msg::Path path;
    GetDistanceAndPathBetweenCells(cur_robot_cell_ind_, i, distance, path, keypose_graph);
    if (distance > 0.0 && path.poses.size() > 1)
    {
      all_cell_indices.push_back(i);
      reachable_exploring_cell_indices.push_back(i);
      distance_matrix_msg_.cell_or_robot_ids.push_back(cell.id);
      distance_matrix_msg_.is_node_robot.push_back(false);
    }
  }

  distance_matrix_msg_.cells = cells;
  distance_matrix_msg_.robots = robots;

  /****** Return home ******/
  if (reachable_exploring_cell_indices.empty())
  {
    exploration_path_ns::ExplorationPath global_path;
    return_home_ = true;

    geometry_msgs::msg::Point home_position = keypose_graph->GetFirstKeyposePosition();
    int home_position_cell_ind_ = GetCellInd(home_position.x, home_position.y, home_position.z);
    double return_home_distance;
    nav_msgs::msg::Path return_home_path;
    GetDistanceAndPathBetweenCells(cur_robot_cell_ind_, home_position_cell_ind_, return_home_distance, return_home_path, keypose_graph);
    if (return_home_distance <= 0.0 || return_home_path.poses.size() < 2)
    {
      // Robot already home or home unreachable
      return global_path;
    }
    global_path.FromPath(return_home_path);
    for (int i = 1; i < global_path.nodes_.size() - 1; i++)
    {
      global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
    }
    global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
    // Make it a loop
    for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
    {
      global_path.Append(global_path.nodes_[i]);
    }
    return global_path;
  }

  return_home_ = false;

  // Construct distance matrix
  int num_agents = other_robot_positions.size() + 1;
  int num_nodes = reachable_exploring_cell_indices.size();
  // RCLCPP_INFO(rclcpp::get_logger("DEBUG"), "num_agents: %d, num_nodes: %d", num_agents, num_nodes);
  int distance_matrix_size = num_nodes + num_agents + 1; // +1 for depot
  std::vector<std::vector<int>> distance_matrix(distance_matrix_size, std::vector<int>(distance_matrix_size, 0));

  for (int i = 0; i < distance_matrix_size - 1; i++)
  {
    for (int j = 0; j < i; j++)
    {
      nav_msgs::msg::Path path;
      double distance;
      GetDistanceAndPathBetweenCells(all_cell_indices[i], all_cell_indices[j], distance, path, keypose_graph);
      distance_matrix[i+1][j+1] = (int) 10 * distance;
      if (distance_matrix[i+1][j+1] <= 0)
      {
        distance_matrix[i+1][j+1] = 1000000;
      }
      global_plan_interfaces::msg::ConnectionBetweenNodes connection;
      connection.from_node_id = i;
      connection.is_from_node_robot = true ? i < num_agents : false;
      connection.to_node_id = j;
      connection.is_to_node_robot = true ? j < num_agents : false;
      connection.distance = distance_matrix[i+1][j+1];
      connection.path = path;
      distance_matrix_msg_.connections.push_back(connection);
    }
  }
  for (int i = 0; i < distance_matrix_size; i++)
  {
    for (int j = i + 1; j < distance_matrix_size; j++)
    {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  /****** Solve the TSP ******/
  // RCLCPP_INFO(rclcpp::get_logger("DEBUG"), "Solve the VRP");
  vrp_solver_ns::DataModel data_model;
  data_model.distance_matrix = distance_matrix;
  data_model.num_vehicles = num_agents;
  std::vector<operations_research::RoutingIndexManager::NodeIndex> starts;
  std::vector<operations_research::RoutingIndexManager::NodeIndex> ends;
  for (int i = 0; i < num_agents; i++)
  {
    starts.push_back(operations_research::RoutingIndexManager::NodeIndex{i + 1});
    ends.push_back(operations_research::RoutingIndexManager::NodeIndex{0}); // home position
  }
  data_model.starts = starts;
  data_model.ends = ends;

  vrp_solver_ns::VRPSolver vrp_solver(data_model);
  std::vector<std::vector<int>> solution = vrp_solver.Solve();

  // Update exploring status
  if (solution[0].size() > 1)
  {
    for (int j = 1; j < solution[0].size() - 1; j++)
    {
      int cell_ind = all_cell_indices[solution[0][j] - 1];
      subspaces_->GetCell(cell_ind).SetExploringStatus(ExploringStatus::InGlobalPlan);
    }
    int cell_ind = all_cell_indices[solution[0][1] - 1];
    subspaces_->GetCell(cell_ind).SetExploringStatus(ExploringStatus::NextInGlobalPlan);
  }
  for (int i = 1; i < num_agents; i++)
  {
    if (solution[i].size() > 1)
    {
      for (int j = 1; j < solution[i].size() - 1; j++)
      {
        int cell_ind = all_cell_indices[solution[i][j] - 1];
        subspaces_->GetCell(cell_ind).SetExploringStatus(ExploringStatus::InOtherGlobalPlan);
      }
    }
  }
  exploring_status_updated_ = true;

  std::vector<int> node_index;
  node_index = solution[0];
  if (node_index.size() <= 1)
  {
    for (int i = 1; i < num_agents; i++)
    {
      for (int j = 0; j < solution[i].size() - 1; j++)
      {
        node_index.push_back(solution[i][j]);
      }
    }
  }

  ordered_cell_indices.clear();

  // Add the first node in the end to make it a loop
  if (!node_index.empty())
  {
    node_index.push_back(node_index[0]);
  }

  // Extract VRP solution
  // RCLCPP_INFO(rclcpp::get_logger("DEBUG"), "Extract VRP solution");
  exploration_path_ns::ExplorationPath global_path;
  Eigen::Vector3d cur_position;
  int cur_ind, next_ind;
  int cur_cell_ind, next_cell_ind;

  for (int i = 0; i < node_index.size() - 1; i++)
  {
    cur_ind = node_index[i] -  1;
    next_ind = node_index[i + 1] - 1;
    cur_cell_ind = all_cell_indices[cur_ind];
    next_cell_ind = all_cell_indices[next_ind];
    cur_position = subspaces_->GetCell(cur_cell_ind).GetRoadmapConnectionPoint();

    double distance;
    nav_msgs::msg::Path path;
    GetDistanceAndPathBetweenCells(cur_cell_ind, next_cell_ind, distance, path, keypose_graph);
    path = misc_utils_ns::SimplifyPath(path);
    if (path.poses.size() < 2)
    {
      continue;
    }

    exploration_path_ns::Node node(cur_position);
    if (i == 0)
    {
      node.type_ = exploration_path_ns::NodeType::ROBOT;
    }
    else
    {
      node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
    }
    if (cur_ind >= num_agents)
    {
      node.global_subspace_index_ = all_cell_indices[cur_ind];
      ordered_cell_indices.push_back(all_cell_indices[cur_ind]);
    }
    else
    {
      node.global_subspace_index_ = -1;
      ordered_cell_indices.push_back(-1);
    }
    global_path.Append(node);

    // Fill in the path in between
    if (path.poses.size() >= 2)
    {
      for (int j = 1; j < path.poses.size() - 1; j++)
      {
        geometry_msgs::msg::Point node_position;
        node_position = path.poses[j].pose.position;
        exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
        keypose_node.keypose_graph_node_ind_ = static_cast<int>(path.poses[j].pose.orientation.x);
        global_path.Append(keypose_node);
      }
    }
  }
  // Append the robot node to the end
  if (!global_path.nodes_.empty())
  {
    global_path.Append(global_path.nodes_[0]);
  }
  distance_matrix_msg_.vrp_solution = global_path.GetPath();
  return global_path;
}

exploration_path_ns::ExplorationPath GridWorld::SolveGlobalTSP(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices, const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  /****** Get the node on keypose graph associated with the robot position *****/
  double min_dist_to_robot = DBL_MAX;
  geometry_msgs::msg::Point global_path_robot_position = robot_position_;
  Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  // Get nearest connected node
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  keypose_graph->GetClosestConnectedNodeIndAndDistance(robot_position_, closest_node_ind, closest_node_dist);
  if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 && closest_node_ind < keypose_graph->GetNodeNum())
  {
    global_path_robot_position = keypose_graph->GetNodePosition(closest_node_ind);
  }
  else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeNum())
  {
    // RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "GridWorld::SolveGlobalTSP: using nearest keypose node for
    // robot position");
    global_path_robot_position = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
  }
  else
  {
    // RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "GridWorld::SolveGlobalTSP: using neighbor cell roadmap
    // connection points for robot position");
    for (int i = 0; i < neighbor_cell_indices_.size(); i++)
    {
      int cell_ind = neighbor_cell_indices_[i];
      if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
      {
        Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
        if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
        {
          double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            global_path_robot_position.x = roadmap_connection_point.x();
            global_path_robot_position.y = roadmap_connection_point.y();
            global_path_robot_position.z = roadmap_connection_point.z();
          }
        }
      }
    }
  }

  /****** Get all the connected exploring cells *****/
  exploration_path_ns::ExplorationPath global_path;
  std::vector<geometry_msgs::msg::Point> exploring_cell_positions;
  std::vector<int> exploring_cell_indices;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end() ||
          (subspaces_->GetCell(i).GetViewPointIndices().empty() && subspaces_->GetCell(i).GetVisitCount() > 1))
      {
        if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
        {
          // Use straight line connection
          exploring_cell_positions.push_back(GetCellPosition(i));
          exploring_cell_indices.push_back(i);
        }
        else
        {
          Eigen::Vector3d connection_point = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
          geometry_msgs::msg::Point connection_point_geo;
          connection_point_geo.x = connection_point.x();
          connection_point_geo.y = connection_point.y();
          connection_point_geo.z = connection_point.z();

          bool reachable = false;
          if (keypose_graph->IsPositionReachable(connection_point_geo))
          {
            reachable = true;
          }
          else
          {
            // Check all the keypose graph nodes within this cell to see if there are any connected nodes
            double min_dist = DBL_MAX;
            double min_dist_node_ind = -1;
            for (const auto& node_ind : subspaces_->GetCell(i).GetGraphNodeIndices())
            {
              geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);
              double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                  node_position, connection_point_geo);
              if (dist < min_dist)
              {
                min_dist = dist;
                min_dist_node_ind = node_ind;
              }
            }
            if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
            {
              reachable = true;
              connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            }
          }
          if (reachable)
          {
            exploring_cell_positions.push_back(connection_point_geo);
            exploring_cell_indices.push_back(i);
          }
        }
      }
    }
  }

  /****** Return home ******/
  if (exploring_cell_indices.empty())
  {
    return_home_ = true;

    geometry_msgs::msg::Point home_position;

    nav_msgs::msg::Path return_home_path;
    if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
    {
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.pose.position = robot_position_;

      geometry_msgs::msg::PoseStamped home_pose;
      home_pose.pose.position = home_position;
      return_home_path.poses.push_back(robot_pose);
      return_home_path.poses.push_back(home_pose);
    }
    else
    {
      home_position = keypose_graph->GetFirstKeyposePosition();
      keypose_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path, false);
      if (return_home_path.poses.size() >= 2)
      {
        global_path.FromPath(return_home_path);
        global_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT;

        for (int i = 1; i < global_path.nodes_.size() - 1; i++)
        {
          global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
        }
        global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
        // Make it a loop
        for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
        {
          global_path.Append(global_path.nodes_[i]);
        }
      }
      else
      {
        // RCLCPP_ERROR(this->get_logger(), "Cannot find path home");
        // TODO: find a path
      }
    }
    return global_path;
  }

  return_home_ = false;

  // Put the current robot position in the end
  exploring_cell_positions.push_back(global_path_robot_position);
  exploring_cell_indices.push_back(-1);

  /******* Construct the distance matrix *****/
  std::vector<std::vector<int>> distance_matrix(exploring_cell_positions.size(),
                                                std::vector<int>(exploring_cell_positions.size(), 0));
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
      {
        // Use straight line connection
        distance_matrix[i][j] =
            static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                      exploring_cell_positions[i], exploring_cell_positions[j]));
      }
      else
      {
        // Use keypose graph
        nav_msgs::msg::Path path_tmp;
        distance_matrix[i][j] =
            static_cast<int>(10 * keypose_graph->GetShortestPath(exploring_cell_positions[i],
                                                                 exploring_cell_positions[j], false, path_tmp, false));
      }
    }
  }

  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = i + 1; j < exploring_cell_positions.size(); j++)
    {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  /****** Solve the TSP ******/
  tsp_solver_ns::DataModel data_model;
  data_model.distance_matrix = distance_matrix;
  data_model.depot = exploring_cell_positions.size() - 1;

  tsp_solver_ns::TSPSolver tsp_solver(data_model);
  tsp_solver.Solve();
  std::vector<int> node_index;
  tsp_solver.getSolutionNodeIndex(node_index, false);

  ordered_cell_indices.clear();

  // Add the first node in the end to make it a loop
  if (!node_index.empty())
  {
    node_index.push_back(node_index[0]);
  }

  if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
  {
    for (int i = 0; i < node_index.size(); i++)
    {
      int cell_ind = node_index[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = exploring_cell_positions[cell_ind];
      exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                     exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
      node.global_subspace_index_ = exploring_cell_indices[cell_ind];
      global_path.Append(node);
      ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]);
    }
  }
  else
  {
    geometry_msgs::msg::Point cur_position;
    geometry_msgs::msg::Point next_position;
    int cur_keypose_id;
    int next_keypose_id;
    int cur_ind;
    int next_ind;

    for (int i = 0; i < node_index.size() - 1; i++)
    {
      cur_ind = node_index[i];
      next_ind = node_index[i + 1];
      cur_position = exploring_cell_positions[cur_ind];
      next_position = exploring_cell_positions[next_ind];

      nav_msgs::msg::Path keypose_path;
      keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false);

      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      if (i == 0)
      {
        node.type_ = exploration_path_ns::NodeType::ROBOT;
      }
      else
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
      }
      node.global_subspace_index_ = exploring_cell_indices[cur_ind];
      global_path.Append(node);

      ordered_cell_indices.push_back(exploring_cell_indices[cur_ind]);

      // Fill in the path in between
      if (keypose_path.poses.size() >= 2)
      {
        for (int j = 1; j < keypose_path.poses.size() - 1; j++)
        {
          geometry_msgs::msg::Point node_position;
          node_position = keypose_path.poses[j].pose.position;
          exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
          keypose_node.keypose_graph_node_ind_ = static_cast<int>(keypose_path.poses[i].pose.orientation.x);
          global_path.Append(keypose_node);
        }
      }
    }
    // Append the robot node to the end
    if (!global_path.nodes_.empty())
    {
      global_path.Append(global_path.nodes_[0]);
    }
  }

  // std::cout << "path order: ";
  // for (int i = 0; i < ordered_cell_indices.size(); i++)
  // {
  //   std::cout << ordered_cell_indices[i] << " -> ";
  // }
  // std::cout << std::endl;

  return global_path;
}

int GridWorld::GetCellKeyposeID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetKeyposeID();
}

void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
{
  viewpoint_positions.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end())
    {
      viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
    }
  }
}

void GridWorld::AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                       const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // For each cell in local planning, horizon, iterate through all neighboring cells
  // and find the shortest path between them from the viewpoint manager (if no direct keypose graph connection exists)
  for (int from_cell_ind = 0; from_cell_ind < subspaces_->GetCellNumber(); from_cell_ind++)
  {
    int viewpoint_num = subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size();
    if (viewpoint_num == 0)
    {
      continue;
    }
    Eigen::Vector3d from_cell_roadmap_connection_position =
        subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
    if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
    {
      continue;
    }
    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> nearby_cell_indices;
    GetNeighborCellIndices(from_cell_sub, Eigen::Vector3i(1, 1, 1), nearby_cell_indices);

    for (int to_cell_ind: nearby_cell_indices)
    {
      if (subspaces_->GetCell(to_cell_ind).GetViewPointIndices().empty())
      {
        continue;
      }
      Eigen::Vector3d to_cell_roadmap_connection_position =
          subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
      if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
      {
        continue;
      }

      bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
          keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);
      if (connected_in_keypose_graph)
      {
        subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
        subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
        continue;
      }

      nav_msgs::msg::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
          from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      if (PathValid(path_in_between, from_cell_ind, to_cell_ind))
      {
        path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
        for (auto& pose : path_in_between.poses)
        {
          pose.pose.orientation.w = -1;
        }
        keypose_graph->AddPath(path_in_between);
        bool connected = HasDirectKeyposeGraphConnection(keypose_graph, from_cell_roadmap_connection_position,
                                                         to_cell_roadmap_connection_position);
        if (!connected)
        {
          subspaces_->GetCell(from_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(to_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(from_cell_ind).RemoveCellConnection(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).RemoveCellConnection(from_cell_ind);
          continue;
        }
        else
        {
          subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
        }
      }
    }
  }
  UpdateCellKeyposeGraphNodes(keypose_graph);
}

bool GridWorld::PathValid(const nav_msgs::msg::Path& path, int from_cell_ind, int to_cell_ind)
{
  if (path.poses.size() >= 2)
  {
    for (const auto& pose : path.poses)
    {
      int cell_ind = GetCellInd(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool GridWorld::HasDirectKeyposeGraphConnection(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                                const Eigen::Vector3d& start_position,
                                                const Eigen::Vector3d& goal_position)
{
  if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
  {
    return false;
  }

  // Search a path connecting start_position and goal_position with a max path length constraint
  geometry_msgs::msg::Point geo_start_position;
  geo_start_position.x = start_position.x();
  geo_start_position.y = start_position.y();
  geo_start_position.z = start_position.z();

  geometry_msgs::msg::Point geo_goal_position;
  geo_goal_position.x = goal_position.x();
  geo_goal_position.y = goal_position.y();
  geo_goal_position.z = goal_position.z();

  double max_path_length = kCellSize * 2;
  nav_msgs::msg::Path path;
  bool found_path =
      keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position, max_path_length, false, path);
  return found_path;
}

}  // namespace grid_world_ns
