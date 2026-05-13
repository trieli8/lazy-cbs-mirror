#include <lazycbs/compute_heuristic.h>
#include <cassert>
#include <cstring>
#include <cfloat>
namespace lazycbs{
ComputeHeuristic::ComputeHeuristic(int goal_location, const bool* my_map, int map_size,
                                   const int* actions_offset, double e_weight, const EgraphReader* egr) :
    my_map (my_map), actions_offset(actions_offset) {
  this->egr = egr;
  this->e_weight = e_weight;
  this->goal_location = goal_location;
  this->map_size = map_size;
  h_vals = new double[map_size];
  for (int i = 0; i < map_size; i++)
    h_vals[i] = DBL_MAX;
  // The current build uses a plain reverse BFS over the grid, which keeps the
  // heuristic code simple and avoids depending on the e-graph in this path.
  std::vector<int> queue;
  unsigned int qhead(0);

  assert(!my_map[goal_location]);
  h_vals[goal_location] = 0.0;
  queue.push_back(goal_location);
  for(; qhead < queue.size(); ++qhead) {
    int loc(queue[qhead]); 
    double d(h_vals[loc] + 1);
    for (int direction = 0; direction < 5; direction++) {
      int next_id = loc + actions_offset[direction];
      if(my_map[next_id])
        continue;
      if(h_vals[next_id] == DBL_MAX) {
        h_vals[next_id] = d;
        queue.push_back(next_id);
      }
    }
  }
}

double* ComputeHeuristic::getHVals() {
  double* retVal = new double [ this->map_size ];
  memcpy (retVal, this->h_vals, sizeof(double) * this->map_size );
  return retVal;
}

ComputeHeuristic::~ComputeHeuristic() {
  delete[] this->h_vals;
  delete[] my_map;
}
}
