#pragma once

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tare_planner_interfaces/srv/merge_planning_interface.hpp>
#include <tare_planner_interfaces/msg/merger_response.hpp>
#include <keypose_graph/keypose_graph.h>
#include <viewpoint_manager/viewpoint_manager.h>

class PlanningInterfaceMerger : public rclcpp::Node
{
public:
  PlanningInterfaceMerger();
  ~PlanningInterfaceMerger() = default;
  bool initialize();

private:
  // Parameters
  int numRobots;
  std::string mergeServiceName;
  std::string mergerResponseTopic;
  int delayInSeconds;
  std::vector<std::shared_ptr<keypose_graph_ns::KeyposeGraph>> keyposeGraphs;
  std::vector<std::shared_ptr<viewpoint_manager_ns::ViewPointManager>> viewpointManagers;
  std::vector<int> uncoveredPointNumbers;
  std::vector<int> uncoveredFrontierPointNumbers;

  // Publishers
  rclcpp::Publisher<tare_planner_interfaces::msg::MergerResponse>::SharedPtr mergerResponsePublisher;
  // Services
  rclcpp::Service<tare_planner_interfaces::srv::MergePlanningInterface>::SharedPtr mergePlanningInterfaceService;
  void handleMergePlanningInterface(
    const std::shared_ptr<tare_planner_interfaces::srv::MergePlanningInterface::Request> request,
    std::shared_ptr<tare_planner_interfaces::srv::MergePlanningInterface::Response> response
  );


};
