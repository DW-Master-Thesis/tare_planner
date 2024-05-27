#include "../../include/tsp_with_cost_solver/tsp_with_cost_solver.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace tsp_with_cost_solver_ns
{
TSPSolver::TSPSolver(const DataModel& data)
    : data_(data)
{
  num_nodes_ = data.distance_matrix.size();
  std::vector<operations_research::RoutingIndexManager::NodeIndex> starts;
  std::vector<operations_research::RoutingIndexManager::NodeIndex> ends;
  starts.push_back(operations_research::RoutingIndexManager::NodeIndex(1));
  ends.push_back(operations_research::RoutingIndexManager::NodeIndex(0));
  manager_ = std::make_shared<RoutingIndexManager>(num_nodes_, 1, starts, ends);
  routing_ = std::make_shared<RoutingModel>(*manager_);

  auto distance_callback = [this](int64_t from_index, int64_t to_index) -> int64_t
  {
    const int from_node = manager_->IndexToNode(from_index).value();
    const int to_node = manager_->IndexToNode(to_index).value();
    return data_.distance_matrix[from_node][to_node];
  };
  auto distance_and_reward_callback = [this](int64_t from_index, int64_t to_index) -> int64_t
  {
    const int from_node = manager_->IndexToNode(from_index).value();
    const int to_node = manager_->IndexToNode(to_index).value();
    int combined_rew = data_.distance_matrix[from_node][to_node] - data_.rewards[to_node] / 10;
    std::cout << "from_node: " << from_node << " to_node: " << to_node << " distance: " << data_.distance_matrix[from_node][to_node] << " reward: " << data_.rewards[to_node] << " combined: " << combined_rew << std::endl;
    return combined_rew;
  };

  const int distance_callback_index = routing_->RegisterTransitCallback(distance_callback);
  const int transit_callback_index = routing_->RegisterTransitCallback(distance_and_reward_callback);
  routing_->AddDimension(distance_callback_index, 0, 1000, true, "distance");
  routing_->AddDimension(transit_callback_index, 0, 10000000, true, "distance_and_reward");
  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Add penalty for dropping nodes
  for (int i = 1; i < num_nodes_; ++i)
  {
    routing_->AddDisjunction(
      {manager_->NodeToIndex(
        RoutingIndexManager::NodeIndex(i)
      )}, 1000
    );
  }
}

std::vector<int> TSPSolver::Solve()
{
  // Set solver parameter
  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  search_parameters.mutable_time_limit()->set_nanos(80000000);

  const Assignment* solution = routing_->SolveWithParameters(search_parameters);
  std::vector<int> route;
  for (int64_t node = routing_->Start(0); !routing_->IsEnd(node); node = solution->Value(routing_->NextVar(node)))
  {
    route.push_back(manager_->IndexToNode(node).value());
  }
  return route;
}
}
