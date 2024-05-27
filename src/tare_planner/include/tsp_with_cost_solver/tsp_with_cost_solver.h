#pragma once

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"

using namespace operations_research;

namespace tsp_with_cost_solver_ns
{
struct DataModel;
class TSPSolver;
}

struct tsp_with_cost_solver_ns::DataModel
{
    std::vector<std::vector<int>> distance_matrix;
    std::vector<int> rewards;
};

class tsp_with_cost_solver_ns::TSPSolver
{
public:
  TSPSolver(const DataModel& data);
  ~TSPSolver() = default;
  std::vector<int> Solve();

private:
  DataModel data_;
  int num_nodes_;
  std::shared_ptr<RoutingIndexManager> manager_;
  std::shared_ptr<RoutingModel> routing_;
};
