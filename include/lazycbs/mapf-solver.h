#ifndef GEAS_MAPF__SOLVER_H
#define GEAS_MAPF__SOLVER_H
// ECBS includes
#include <lazycbs/map_loader.h>
#include <lazycbs/agents_loader.h>

// geas includes
#include <geas/solver/solver.h>
#include <geas/utils/bitset.h>

#include <lazycbs/agent-pf.h>
#include <unordered_map>

namespace lazycbs {

class MAPF_Solver {
 public:
  struct SolveAborted { };

  struct penalty {
    geas::pid_t p;
    geas::pval_t lb;
  };
  
  struct cons_key {
    int timestamp;
    int loc1;
    int loc2;
  };
  struct cons_key_hasher {
    size_t operator()(const cons_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.timestamp)^h;
      h = ((h<<5) + k.loc1)^h;
      h = ((h<<5) + k.loc2)^h;
      return h;
    }
  };
  struct cons_key_eq {
    bool operator()(const cons_key& x, const cons_key& y) const {
      return x.timestamp == y.timestamp && x.loc1 == y.loc1 && x.loc2 == y.loc2;
    }
  };
    
  enum BarrierDir { UP = 0, LEFT = 1, DOWN = 2, RIGHT = 3 };

  struct barrier_key {
    int agent;
    BarrierDir dir; 
    int location;
    int time_at_edge;
  };
  struct barrier_data {
    geas::patom_t act; 
    int start;
    int duration;
  };

  struct barrier_key_hasher {
    size_t operator()(const barrier_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.agent)^h;
      h = ((h<<5) + (size_t) k.dir)^h;
      h = ((h<<5) + k.location)^h;
      h = ((h<<5) + k.time_at_edge)^h;
      return h;
    }
  };
  struct barrier_key_eq {
    bool operator()(const barrier_key& x, const barrier_key& y) const {
      return x.agent == y.agent && x.dir == y.dir && x.location == y.location && x.time_at_edge == y.time_at_edge;
    }
  };

  struct target_key {
    int timestamp;
    int target_agent;
    int moving_agent;
    int location;
  };
  struct target_key_hasher {
    size_t operator()(const target_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.timestamp)^h;
      h = ((h<<5) + k.target_agent)^h;
      h = ((h<<5) + k.moving_agent)^h;
      h = ((h<<5) + k.location)^h;
      return h;
    }
  };
  struct target_key_eq {
    bool operator()(const target_key& x, const target_key& y) const {
      return x.timestamp == y.timestamp && x.target_agent == y.target_agent &&
        x.moving_agent == y.moving_agent && x.location == y.location;
    }
  };

  enum ConflictType { C_MUTEX, C_BARRIER, C_TARGET };
  struct conflict_key {
    ConflictType type;
    int timestamp;
    int a1;
    int a2;
    int loc1;
    int loc2;
  };
  struct conflict_key_hasher {
    size_t operator()(const conflict_key& k) const {
      size_t h(5331);
      h = ((h<<5) + static_cast<size_t>(k.type))^h;
      h = ((h<<5) + k.timestamp)^h;
      h = ((h<<5) + k.a1)^h;
      h = ((h<<5) + k.a2)^h;
      h = ((h<<5) + k.loc1)^h;
      h = ((h<<5) + k.loc2)^h;
      return h;
    }
  };
  struct conflict_key_eq {
    bool operator()(const conflict_key& x, const conflict_key& y) const {
      return x.type == y.type && x.timestamp == y.timestamp &&
        x.a1 == y.a1 && x.a2 == y.a2 && x.loc1 == y.loc1 && x.loc2 == y.loc2;
    }
  };
  struct barrier_info {
    int s_loc; // Start corner
    int e_loc; // Exit corner
  };
  struct conflict {
    conflict(void) { }

    conflict(int _timestamp, int _a1, int _a2, int _loc1, int _loc2)
      : timestamp(_timestamp), type(C_MUTEX)
      , a1(_a1), a2(_a2), p({_loc1, _loc2}) { }

    conflict(int _timestamp, int _a1, int _a2, int _loc1, int _loc2, bool dummy)
      : timestamp(_timestamp), type(C_BARRIER)
      , a1(_a1), a2(_a2), b({_loc1, _loc2}) { }
    /*
    conflict(int _timestamp, int _a1, int _a2, int h_loc, int v_loc, int r_loc)
      : timestamp(_timestamp), type(C_BARRIER)
      , a1(_a1), a2(_a2), b({h_loc, v_loc, r_loc}) { }
      */

    static conflict barrier(int t, int a1, int a2, int loc1, int loc2) {
      conflict c(t, a1, a2, loc1, loc2, false);
      return c;
    }

    static conflict target(int t, int a1, int a2, int loc1, int loc2) {
      conflict c;
      c.timestamp = t;
      c.type = C_TARGET;
      c.a1 = a1;
      c.a2 = a2;
      c.p.loc1 = loc1;
      c.p.loc2 = loc2;
      return c;
    }

    int timestamp;
    ConflictType type;

    int a1;
    int a2;
     
    union {
      struct {
        int loc1;
        int loc2;
      } p;
      barrier_info b;
    };
  };

  struct cons_data {
    intvar sel; // Selector variable
    btset::bitset attached; // Which agents are already attached?
  };
  struct target_data {
    patom_t sel; // Branch selector for the target-symmetry split.
    bool attached;
  };

  MAPF_Solver(const  MapLoader& ml, const  AgentsLoader& al, const  EgraphReader& egr, int cost_ub);
  MAPF_Solver(const  MapLoader& ml, const  AgentsLoader& al, const  EgraphReader& egr, int cost_ub, bool verbose);
  MAPF_Solver(const  MapLoader& ml, const  AgentsLoader& al, const  EgraphReader& egr, int cost_ub, bool verbose, bool super_verbose);
  MAPF_Solver(const  MapLoader& ml, const  AgentsLoader& al, const  EgraphReader& egr, int cost_ub, bool verbose, bool super_verbose, bool enable_target_symmetry);
  MAPF_Solver(const  MapLoader& ml, const  AgentsLoader& al, const  EgraphReader& egr, int cost_ub, bool verbose, bool super_verbose, bool enable_target_symmetry, bool enable_conflict_tiebreaker);

  // Problem information
  const  MapLoader* ml;
  const  AgentsLoader* al;
  const  EgraphReader* egr;
  const int map_size;

  // Solver engine
  geas::solver s;

  // Single-agent search engines
  geas::vec<Agent_PF*> pathfinders;

  // For conflict checking
  geas::vec<bool> reservation_table;
  geas::vec<int> cmap; // Map at the current time
  geas::vec<int> nmap; // Map at the next time

  // Constraints?
  geas::vec<cons_data> constraints;
  geas::vec< geas::vec<barrier_data> > barriers;
  ::std::unordered_map<cons_key, int, cons_key_hasher, cons_key_eq> cons_map;
  ::std::unordered_map<barrier_key, int, barrier_key_hasher, barrier_key_eq> barrier_map;
  ::std::unordered_map<target_key, int, target_key_hasher, target_key_eq> target_map;
  geas::vec<target_data> target_constraints;
  // conflict new_conflict;
  geas::vec<conflict> new_conflicts;
  ::std::unordered_map<conflict_key, unsigned int, conflict_key_hasher, conflict_key_eq> conflict_table;
  p_sparseset agent_set;

  // For unsat-core reasoning
  geas::vec<penalty> penalties;
  ::std::unordered_map<geas::pid_t, int> penalty_table;
  int cost_lb;
  int cost_ub;
  bool verbose;
  bool super_verbose;
  bool enable_target_symmetry;
  bool enable_conflict_tiebreaker;

  // How many high-level conflicts have been processed?
  int HL_conflicts;

  inline int row_of(int loc) const { return loc / ml->cols; }
  inline int col_of(int loc) const { return loc % ml->cols; }

  int maxPathLength(void) const;

  // Search for some feasible plan, given a set of assumptions.
  bool buildPlan(geas::vec<geas::patom_t>& assumps);
  bool minimizeCost();
//   bool minimizeMakespan();
  void printPaths(FILE* f = stdout) const;
  void getPaths(::std::pair<int, ::std::vector< ::std::vector< ::std::pair<int, int> > > > *solution) const;
  void printStats(FILE* f = stdout) const;
   
  bool runUCIter(void);
  bool checkForConflicts(void);
  // In buildPlan, we also try to apply bypasses.
  bool resolveConflicts(void);
  bool addConflict(void);
  bool processCore(geas::vec<geas::patom_t>& core);
      
  geas::patom_t getBarrier(int ai, BarrierDir dir, int t0, int p0, int dur);
  geas::patom_t getTargetBarrier(int ai, int t0, int p0, int dur);

  int monotoneSubchainStart(int dy, int dx, int ai, int t) const;
  int monotoneSubchainEnd(int dy, int dx, int ai, int t) const;

  ::std::pair<int, bool*> retrieve_reservation_table(int ai);
  unsigned int conflict_step_bias(int excl, int curr_id, int next_id, int next_timestep) const;
  void tracef(const char* fmt, ...) const;

  ~MAPF_Solver();
};

bool MAPF_MinCost(MAPF_Solver& s);
bool MAPF_MinMakespan(MAPF_Solver& s);
bool MAPF_MaxDeadlines(MAPF_Solver& s);

}

#endif
