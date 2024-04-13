#include "../../include/vrp_solver/vrp_solver.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace vrp_solver_ns
{
VRPSolver::VRPSolver(const DataModel& data_model)
    : data_(data_model)
{
  manager_ = std::make_shared<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.starts, data_.ends);
  routing_ = std::make_shared<RoutingModel>(*manager_);
  const int transit_callback_index = routing_->RegisterTransitCallback(
    [this](int64_t from_index, int64_t to_index) -> int64_t {
      const int from_node = manager_->IndexToNode(from_index).value();
      const int to_node = manager_->IndexToNode(to_index).value();
      return data_.distance_matrix[from_node][to_node];
  });
  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  // Add Distance constraint.
  routing_->AddDimension(transit_callback_index, 0, 9999,
                         /*fix_start_cumul_to_zero=*/true, "Distance");
  routing_->GetMutableDimension("Distance")->SetGlobalSpanCostCoefficient(100);
}

std::vector<std::vector<int>> VRPSolver::Solve()
{
  // Set solver parameter
  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  search_parameters.mutable_time_limit()->set_nanos(80000000);

  const Assignment* solution = routing_->SolveWithParameters(search_parameters);
  std::vector<std::vector<int>> routes;
  for (int vehicle_id = 0; vehicle_id < data_.num_vehicles; vehicle_id++)
  {
    std::vector<int> route;
    for (int64_t node = routing_->Start(vehicle_id); !routing_->IsEnd(node); node = solution->Value(routing_->NextVar(node)))
    {
      route.push_back(manager_->IndexToNode(node).value());
    }
    routes.push_back(route);
  }
  return routes;
}

}
