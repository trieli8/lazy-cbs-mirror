// Represents 2D-Nodes
#ifndef NODE_H
#define NODE_H

#include <boost/heap/fibonacci_heap.hpp>
#include <string>
#include <vector>
#include <functional>  // for std::hash (c++11 and above)

using boost::heap::fibonacci_heap;
using boost::heap::compare;
namespace lazycbs{
class Node {
 public:

  int id;
  double g_val;
  double h_val = 0;
  Node* parent;
  int timestep = 0;
  int num_internal_conf = 0;  // used in ECBS
  bool in_openlist = false;
  double g_hwy_val;      // used in TCBS (Tansel's idea to use EG only in secondary heuristic)
  double h_hwy_val = 0;  // used in TCBS

  ///////////////////////////////////////////////////////////////////////////////
  // NOTE -- Normally, compare_node (lhs,rhs) suppose to return true if lhs<rhs.
  //         However, Heaps in STL and Boost are implemented as max-Heap.
  //         Hence, to achieve min-Head, we return true if lhs>rhs
  ///////////////////////////////////////////////////////////////////////////////
  
  // Compare nodes by f-value, then prefer larger g-values to break ties.
  struct compare_node {
    // returns true if n1 > n2 (note -- this gives us *min*-heap).
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
        return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
      return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
    }
  };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

  // Compare nodes in FOCAL by conflicts first, then by f-value, then by g-value.
  struct secondary_compare_node {
    // returns true if n1 > n2
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->num_internal_conf == n2->num_internal_conf) {
        if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
          return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
        return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
      }
      return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

  // Compare TCBS nodes by conflicts first, then by highway heuristic.
  struct secondary_hwy_compare_node {
    // returns true if n1 > n2
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->num_internal_conf == n2->num_internal_conf) {
        if (n1->g_hwy_val + n1->h_hwy_val == n2->g_hwy_val + n2->h_hwy_val)
          return n1->g_hwy_val <= n2->g_hwy_val;  // break ties towards larger g_HWY_vals
        return n1->g_hwy_val + n1->h_hwy_val >= n2->g_hwy_val + n2->h_hwy_val;  // break ties towards smaller f_vals (prefer shorter solutions)
      }
      return n1->num_internal_conf >= n2->num_internal_conf;  // prefer less conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

  // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
  typedef boost::heap::fibonacci_heap< Node* , compare<compare_node> >::handle_type open_handle_t;
  typedef boost::heap::fibonacci_heap< Node* , compare<secondary_compare_node> >::handle_type focal_handle_t;  // ECBS
  typedef boost::heap::fibonacci_heap< Node* , compare<secondary_hwy_compare_node> >::handle_type tansel_focal_handle_t;  // TCBS

  open_handle_t open_handle;
  focal_handle_t focal_handle;
  tansel_focal_handle_t tansel_focal_handle;

  Node();
  Node(const Node& other);
  Node(int id, double g_val, double h_val, Node* parent, int timestep, int num_internal_conf = 0, bool in_openlist = false, double g_hwy_val = 0, double h_hwy_val = 0);
  inline double getFVal() const { return g_val + h_val; }
  ~Node();

  // The following is used by googledensehash for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and agree on the id and timestep
  struct eqnode {
    bool operator()(const Node* s1, const Node* s2) const {
      return (s1 == s2) || (s1 && s2 && s1->id == s2->id && s1->timestep == s2->timestep);
    }
  };

  // Hash nodes by id and timestep so sparse hash tables can store them directly.
  struct NodeHasher {
    std::size_t operator()(const Node* n) const {
      const std::size_t id_hash = std::hash<int>()(n->id);
      const std::size_t timestep_hash = std::hash<int>()(n->timestep);
      return id_hash ^ (timestep_hash << 1);
    }
  };
};

std::ostream& operator<<(std::ostream& os, const Node& n);
}
#endif
