#include <planner_interface_merger/planning_interface_merger.h>
#include <tare_planner_interfaces/msg/detail/planning_interface__struct.hpp>

PlanningInterfaceMerger::PlanningInterfaceMerger() : Node("planning_interface_merger")
{
  // Declare parameters
  this->declare_parameter("num_robots", 1);
  this->declare_parameter("merge_service_name", "merge_planning_interface");
  this->get_parameter("num_robots", numRobots);
  this->get_parameter("merge_service_name", mergeServiceName);

  RCLCPP_INFO(this->get_logger(), "Number of robots: %d", numRobots);
  RCLCPP_INFO(this->get_logger(), "Merge service name: %s", mergeServiceName.c_str());
}

bool PlanningInterfaceMerger::initialize()
{
  // keypose_graph
  this->declare_parameter<double>("keypose_graph/kAddNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeConnectDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeVerticalThreshold", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
  this->declare_parameter<int>("keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);
  // viewpoint_manager
  this->declare_parameter<int>("viewpoint_manager/number_x", 80);
  this->declare_parameter<int>("viewpoint_manager/number_y", 80);
  this->declare_parameter<int>("viewpoint_manager/number_z", 40);
  this->declare_parameter<double>("viewpoint_manager/resolution_x", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_y", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_z", 0.5);
  this->declare_parameter<double>("kConnectivityHeightDiffThr", 0.25);
  this->declare_parameter<double>("kViewPointCollisionMargin", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZPlus", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZMinus", 0.5);
  this->declare_parameter<double>("kCollisionGridZScale", 2.0);
  this->declare_parameter<double>("kCollisionGridResolutionX", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionY", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionZ", 0.5);
  this->declare_parameter<bool>("kLineOfSightStopAtNearestObstacle", true);
  this->declare_parameter<bool>("kCheckDynamicObstacleCollision", true);
  this->declare_parameter<int>("kCollisionFrameCountMax", 3);
  this->declare_parameter<double>("kViewPointHeightFromTerrain", 0.75);
  this->declare_parameter<double>("kViewPointHeightFromTerrainChangeThreshold", 0.6);
  this->declare_parameter<int>("kCollisionPointThr", 3);
  this->declare_parameter<double>("kCoverageOcclusionThr", 1.0);
  this->declare_parameter<double>("kCoverageDilationRadius", 1.0);
  this->declare_parameter<double>("kCoveragePointCloudResolution", 1.0);
  this->declare_parameter<double>("kSensorRange", 10.0);
  this->declare_parameter<double>("kNeighborRange", 3.0);
  // Initialize keyposeGraphs, viewpointManagers, uncoveredPointNumbers, and uncoveredFrontierPointNumbers
  keyposeGraphs.clear();
  viewpointManagers.clear();
  uncoveredPointNumbers.clear();
  uncoveredFrontierPointNumbers.clear();
  for (int i = 0; i < numRobots; i++)
  {
    keyposeGraphs.push_back(std::make_shared<keypose_graph_ns::KeyposeGraph>(shared_from_this()));
    viewpointManagers.push_back(std::make_shared<viewpoint_manager_ns::ViewPointManager>(shared_from_this()));
    uncoveredPointNumbers.push_back(0);
    uncoveredFrontierPointNumbers.push_back(0);
  }

  // Create service
  mergePlanningInterfaceService = this->create_service<tare_planner_interfaces::srv::MergePlanningInterface>(
    mergeServiceName,
    std::bind(&PlanningInterfaceMerger::handleMergePlanningInterface, this, std::placeholders::_1, std::placeholders::_2)
  );
  RCLCPP_INFO(this->get_logger(), "PlanningInterfaceMerger node initialized");
  return true;
}

void PlanningInterfaceMerger::handleMergePlanningInterface(
  const std::shared_ptr<tare_planner_interfaces::srv::MergePlanningInterface::Request> request,
  std::shared_ptr<tare_planner_interfaces::srv::MergePlanningInterface::Response> response
)
{
  int robotId = request->robot_id;
  if (robotId < 0 || robotId >= numRobots)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid robot id: %d", robotId);
    return;
  }
  auto keyposeGraph = request->planning_interface.keypose_graph;
  auto viewpointManager = request->planning_interface.viewpoint_manager;
  auto uncoveredPointNumber = request->planning_interface.uncovered_point_num;
  auto uncoveredFrontierPointNumber = request->planning_interface.uncovered_frontier_point_num;
  keyposeGraphs[robotId]->FromMsg(keyposeGraph);
  viewpointManagers[robotId]->FromMsg(viewpointManager);
  uncoveredPointNumbers[robotId] = uncoveredPointNumber;
  uncoveredFrontierPointNumbers[robotId] = uncoveredFrontierPointNumber;

  response->planning_interfaces.clear();
  for (int i = 0; i < numRobots; i++)
  {
    tare_planner_interfaces::msg::PlanningInterface planningInterface;
    planningInterface.keypose_graph = keyposeGraphs[i]->ToMsg();
    planningInterface.viewpoint_manager = viewpointManagers[i]->ToMsg();
    planningInterface.uncovered_point_num = uncoveredPointNumbers[i];
    planningInterface.uncovered_frontier_point_num = uncoveredFrontierPointNumbers[i];
    response->planning_interfaces.push_back(planningInterface);
    response->robot_ids.push_back(i);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningInterfaceMerger>();
  node->initialize();
  RCLCPP_INFO(node->get_logger(), "PlanningInterfaceMerger node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
