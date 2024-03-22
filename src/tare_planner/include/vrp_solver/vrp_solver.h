#pragma once

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"

using namespace operations_research;

namespace vrp_solver_ns
{
struct DataModel;
class VRPSolver;
}

struct vrp_solver_ns::DataModel
{
    int num_vehicles;
    std::vector<std::vector<int>> distance_matrix;
    std::vector<RoutingIndexManager::NodeIndex> starts;
    std::vector<RoutingIndexManager::NodeIndex> ends;
};

class vrp_solver_ns::VRPSolver
{
public:
  VRPSolver(const DataModel& data_model);
  ~VRPSolver() = default;
  std::vector<std::vector<int>> Solve();

private:
  DataModel data_;
  std::shared_ptr<RoutingIndexManager> manager_;
  std::shared_ptr<RoutingModel> routing_;
};
