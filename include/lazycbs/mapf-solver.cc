#ifndef GEAS_MAPF__SOLVER_CPP
#define GEAS_MAPF__SOLVER_CPP

#include <geas/constraints/builtins.h>

#include <lazycbs/mapf-solver.h>

#include <lazycbs/compute_heuristic.h>

#include <cstdarg>

// #define DEBUG_UC
// #define MAPF_NO_RECTANGLES

namespace lazycbs {

// Adapter used by Agent_PF to ask the solver for a fresh reservation table
// without exposing the solver internals directly.
static ::std::pair<int, bool*> mapf_get_res_table(MAPF_Solver* m, int excl) {
  return m->retrieve_reservation_table(excl);
}

static unsigned int mapf_get_step_bias(MAPF_Solver* m, int excl, int curr_id, int next_id, int next_timestep) {
  return m->conflict_step_bias(excl, curr_id, next_id, next_timestep);
}

static MAPF_Solver::conflict_key make_conflict_key(const MAPF_Solver::conflict& c) {
  MAPF_Solver::conflict_key key;
  key.type = c.type;
  key.timestamp = c.timestamp;
  key.a1 = c.a1;
  key.a2 = c.a2;
  key.loc1 = c.p.loc1;
  key.loc2 = c.p.loc2;

  if(c.type == MAPF_Solver::C_MUTEX) {
    if(key.a2 < key.a1)
      ::std::swap(key.a1, key.a2);
    if(key.loc2 >= 0 && key.loc2 > key.loc1)
      ::std::swap(key.loc1, key.loc2);
  }

  return key;
}

static void record_conflict(MAPF_Solver& mapf, const MAPF_Solver::conflict& c) {
  ++mapf.conflict_table[make_conflict_key(c)];
}

MAPF_Solver::MAPF_Solver(const lazycbs::MapLoader& _ml, const lazycbs::AgentsLoader& _al, const lazycbs::EgraphReader& _egr, int UB)
  : MAPF_Solver(_ml, _al, _egr, UB, false) {
}

MAPF_Solver::MAPF_Solver(const lazycbs::MapLoader& _ml, const lazycbs::AgentsLoader& _al, const lazycbs::EgraphReader& _egr, int UB, bool _verbose)
  : MAPF_Solver(_ml, _al, _egr, UB, _verbose, false) {
}

MAPF_Solver::MAPF_Solver(const lazycbs::MapLoader& _ml, const lazycbs::AgentsLoader& _al, const lazycbs::EgraphReader& _egr, int UB, bool _verbose, bool _super_verbose)
  : MAPF_Solver(_ml, _al, _egr, UB, _verbose, _super_verbose, true, true) {
}

MAPF_Solver::MAPF_Solver(const lazycbs::MapLoader& _ml, const lazycbs::AgentsLoader& _al, const lazycbs::EgraphReader& _egr, int UB, bool _verbose, bool _super_verbose, bool _enable_target_symmetry)
  : MAPF_Solver(_ml, _al, _egr, UB, _verbose, _super_verbose, _enable_target_symmetry, true) {
}

MAPF_Solver::MAPF_Solver(const lazycbs::MapLoader& _ml, const lazycbs::AgentsLoader& _al, const lazycbs::EgraphReader& _egr, int UB, bool _verbose, bool _super_verbose, bool _enable_target_symmetry, bool _enable_conflict_tiebreaker)
  : ml(&_ml), al(&_al), egr(&_egr), map_size(ml->rows * ml->cols)
  , reservation_table(map_size, false), cmap(map_size, -1), nmap(map_size, -1)
  , agent_set(al->num_of_agents)
  , cost_ub(UB)
  , HL_conflicts(0)
  , verbose(_verbose)
  , super_verbose(_super_verbose)
  , enable_target_symmetry(_enable_target_symmetry)
  , enable_conflict_tiebreaker(_enable_conflict_tiebreaker) {

    int num_of_agents = al->num_of_agents;
    cost_lb = 0;

    // Build one low-level planner per agent. Each planner gets the same map,
    // a precomputed heuristic to its goal, and a callback for the shared
    // reservation table that excludes the agent currently being queried.
    for(int ai = 0; ai < num_of_agents; ++ai) {
      int init_loc = ml->linearize_coordinate((al->initial_locations[ai]).first, (al->initial_locations[ai]).second);
      int goal_loc = ml->linearize_coordinate((al->goal_locations[ai]).first, (al->goal_locations[ai]).second);
      lazycbs::ComputeHeuristic ch(goal_loc, ml->get_map(), map_size, ml->actions_offset, 1, egr);

      geas::intvar cv(s.new_intvar(0, UB));
      Agent_PF* pf(new Agent_PF(s.data, cv, init_loc, goal_loc, ch.getHVals(), ml->get_map(), map_size, ml->actions_offset,
        ::std::bind(mapf_get_res_table, this, ai),
        ::std::bind(mapf_get_step_bias, this, ai, ::std::placeholders::_1, ::std::placeholders::_2, ::std::placeholders::_3)));
      pathfinders.push(pf);

      cost_lb += pf->pathCost();
      penalty_table.insert(::std::make_pair(cv.p, penalties.size()));
      penalties.push(penalty { cv.p, geas::from_int(pf->pathCost()) });
    }

    tracef("MAPF init: agents=%d map=%dx%d initial_cost_lb=%d cost_ub=%d",
      num_of_agents, ml->rows, ml->cols, cost_lb, cost_ub);
    tracef("MAPF init: target symmetry %s", enable_target_symmetry ? "enabled" : "disabled");
    tracef("MAPF init: conflict tiebreaker %s", enable_conflict_tiebreaker ? "enabled" : "disabled");
}

void MAPF_Solver::tracef(const char* fmt, ...) const {
  if(!(verbose || super_verbose))
    return;

  va_list ap;
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
}

// Get the local reservation table for agent excl.
// This is kind of expensive; find a better way.
::std::pair<int, bool*> MAPF_Solver::retrieve_reservation_table(int excl) {
  // Build a dense [time][cell] table for all other agents, then hand back a
  // raw pointer because the low-level propagators expect contiguous storage.
  // Finished paths are padded at the goal so they behave like stationary
  // obstacles after their last move.
  const int frame_sz = ml->rows * ml->cols;
  int cap = 0;
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    if(ai == excl)
      continue;
    cap = ::std::max(cap, pathfinders[ai]->getPath().size());
  }
  // Clear the existing table. This _should_ be safe (assuming sizeof(bool) == 1).
  ::std::memset(reservation_table.begin(), 0, reservation_table.size());
  reservation_table.growTo(frame_sz * cap, false);

  // Now fill it
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    if(ai == excl)
      continue;
    int base = 0;
    for(int p : pathfinders[ai]->getPath()) {
      reservation_table[base + p] = true;
      base += frame_sz;
    }
    int pE = pathfinders[ai]->getPath().last();
    for(; base < frame_sz * cap; base += frame_sz)
      reservation_table[base + pE] = true;
  }

  return ::std::make_pair(cap, reservation_table.begin());
}

bool apply_penalties(MAPF_Solver& mf) {
   for(MAPF_Solver::penalty& p : mf.penalties)  {
    geas::patom_t at(geas::le_atom(p.p, p.lb));
    if(!mf.s.assume(at))
      return false;
    if(mf.s.is_aborted())
      throw MAPF_Solver::SolveAborted { };
  }
  return true;
}

void log_conflict(MAPF_Solver& mapf) {
  for(auto new_conflict : mapf.new_conflicts) {
    int a1(new_conflict.a1);
    int a2(new_conflict.a2);
    int t(new_conflict.timestamp);

    if(new_conflict.type == MAPF_Solver::C_BARRIER) {
      fprintf(stderr, "%%%% Adding rectangle: [%d, (%d, %d) |- (%d, %d), %d, %d]\n",
        t, mapf.row_of(new_conflict.b.s_loc), mapf.col_of(new_conflict.b.s_loc),
           mapf.row_of(new_conflict.b.e_loc), mapf.col_of(new_conflict.b.e_loc),
           a1, a2);
    } else if(new_conflict.type == MAPF_Solver::C_TARGET) {
      fprintf(stderr, "%%%% Adding target conflict: [%d, agents=(%d,%d), loc=%d]\n",
        t, a1, a2, new_conflict.p.loc1);
    } else {
      fprintf(stderr, "%%%% Adding conflict: [%d, (%d, %d), %d, %d, %d | (%d, %d -> %d, %d) | (%d, %d -> %d, %d) ]\n", new_conflict.timestamp, new_conflict.p.loc1 / mapf.ml->cols, new_conflict.p.loc1 % mapf.ml->cols, new_conflict.p.loc2, new_conflict.a1, new_conflict.a2,
        mapf.pathfinders[new_conflict.a1]->start_pos / mapf.ml->cols, mapf.pathfinders[new_conflict.a1]->start_pos % mapf.ml->cols,
        mapf.pathfinders[new_conflict.a1]->goal_pos / mapf.ml->cols, mapf.pathfinders[new_conflict.a1]->goal_pos % mapf.ml->cols,
        mapf.pathfinders[new_conflict.a2]->start_pos / mapf.ml->cols, mapf.pathfinders[new_conflict.a2]->start_pos % mapf.ml->cols,
        mapf.pathfinders[new_conflict.a2]->goal_pos / mapf.ml->cols, mapf.pathfinders[new_conflict.a2]->goal_pos % mapf.ml->cols);
    }
  }
}

bool MAPF_Solver::buildPlan(geas::vec<geas::patom_t>& assumps) {
  s.clear_assumptions();  
  tracef("buildPlan: solving with %d assumption(s)", (int) assumps.size());
  
  // Load the current bounds into the GEAS solver once, then solve. If the
  // returned plan still contains a MAPF conflict, we add a new constraint and
  // retry from a fresh solver state.
  for(geas::patom_t at : assumps) {
    if(!s.assume(at))
      return false;
    if(s.is_aborted())
      throw MAPF_Solver::SolveAborted { };
  }

retry:
  switch(s.solve()) {
    // We've proven the current subproblem is infeasible.
    case geas::solver::UNSAT:
      return false;
    // Candidate optimal solution. Check for conflicts.
    case geas::solver::SAT:
      // The SAT plan can still violate higher-level MAPF constraints, so try
      // to repair or refine it before accepting the result.
      tracef("buildPlan: SAT, checking for conflicts");
      if(!resolveConflicts()) {
        tracef("buildPlan: conflict resolution failed, adding conflict and retrying");
        s.restart();
#ifdef DEBUG_UC
        log_conflict(*this);
#endif
        if(!addConflict())
          return false;
        goto retry;
      }
      // If we succeeded, done.
      tracef("buildPlan: SAT and conflict-free");
      return true;
    case geas::solver::UNKNOWN:
      throw SolveAborted { };
  }
  // Should be unreachable
  GEAS_ERROR;
}


bool MAPF_Solver::minimizeCost(void) {
  s.clear_assumptions();

  // This is the main optimization loop: solve, discover conflicts, tighten the
  // lower bound using unsat cores, and repeat until the current plan is both
  // feasible and conflict-free.
  cost_lb = 0;
  for(Agent_PF* p : pathfinders) {
    cost_lb += p->pathCost();
  }
  tracef("minimizeCost: initial lower bound %d", cost_lb);
#ifdef DEBUG_UC
  fprintf(stderr, "%%%% Initial bound: %d\n", cost_lb);
#endif
    
  geas::vec<geas::patom_t> core;
  if(!apply_penalties(*this))
    return false;
  while(checkForConflicts()) {
#ifdef DEBUG_UC
    log_conflict(*this);
#endif
    // s.restart();
    if(!addConflict())
      return false;
    
    // s.clear_assumptions();
    while(!runUCIter()) {
      s.get_conflict(core);
      s.clear_assumptions();
      s.restart();
      cost_lb += processCore(core);
      tracef("minimizeCost: core size %d -> lower bound %d", (int) core.size(), cost_lb);
      apply_penalties(*this);
#ifdef DEBUG_UC
      fprintf(stderr, "%%%% Found core of size (%d), current lower bound %d\n", core.size(), cost_lb);
#endif
    }
  }
  return true;
}

void MAPF_Solver::printStats(FILE* f) const {
  int LL_num_generated = 0;
  int LL_num_expanded = 0;
  int LL_executions = 0;
  for(Agent_PF* p : pathfinders) {
    LL_num_generated += p->num_generated;
    LL_num_expanded += p->num_expanded;
    LL_executions += p->num_executions;
  }
  fprintf(f,
    "cost_lb=%d, solver_conflicts=%d, ll_expanded=%d, ll_generated=%d, hl_conflicts=%d, conflict_table=%zu, ll_executions=%d",
    cost_lb, s.data->stats.conflicts, LL_num_expanded, LL_num_generated, HL_conflicts, conflict_table.size(), LL_executions);
  // ::std::cout << "cost_lb=" << cost_lb << ", solver_conflicts=" << s.data->stats.conflicts
  //             << ", ll_expanded=" << LL_num_expanded << ", ll_generated=" << LL_num_generated
  //             << ", hl_conflicts=" << HL_conflicts << ", ll_executions=" << LL_executions;
}

void MAPF_Solver::printPaths(FILE* f) const {
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    fprintf(f, "Agent %d:", ai);
    for(int loc : pathfinders[ai]->getPath()) {
      // fprintf(f, " %d", loc);
      fprintf(f, " (%d,%d)", row_of(loc)-1, col_of(loc)-1);
    }
    fprintf(f, "\n");
  }
}

void MAPF_Solver::getPaths(::std::pair<int, ::std::vector< ::std::vector< ::std::pair<int, int> > > > *solution) const {
  ::std::vector<::std::vector<::std::pair<int, int> > > paths(pathfinders.size());

  for(int ai=0; ai<pathfinders.size(); ++ai){
    paths[ai] = ::std::vector<::std::pair<int, int> >();
    for(int loc: pathfinders[ai]->getPath()){
      paths[ai].push_back(::std::make_pair(row_of(loc)-1, col_of(loc)-1));
    }
  }
  solution->first = cost_lb;
  solution->second = paths;
}

int MAPF_Solver::maxPathLength(void) const {
  int len = 0;
  for(Agent_PF* p : pathfinders)
    len = ::std::max(len, static_cast<int>(::std::ceil(p->pathCost())));
  return len;
}

inline int agentPosition(Agent_PF* p, int t) {
  const geas::vec<int>& P(p->getPath());
  return (t < P.size()) ? P[t] : P.last();
}

inline bool agentStaysAtGoal(Agent_PF* p, int t) {
  const geas::vec<int>& P(p->getPath());
  if(P.size() == 0)
    return false;
  return t >= static_cast<int>(P.size()) - 1 && agentPosition(p, t) == p->goal_pos;
}

unsigned int MAPF_Solver::conflict_step_bias(int excl, int curr_id, int next_id, int next_timestep) const {
  if(!enable_conflict_tiebreaker)
    return 0;
  if(next_timestep <= 0 || excl < 0 || excl >= pathfinders.size())
    return 0;

  auto lookup = [this](const conflict& c) -> unsigned int {
    auto it(conflict_table.find(make_conflict_key(c)));
    return it == conflict_table.end() ? 0u : (*it).second;
  };

  unsigned int score = 0;
  bool excl_at_goal = agentStaysAtGoal(pathfinders[excl], next_timestep);

  for(int aj = 0; aj < pathfinders.size(); ++aj) {
    if(aj == excl)
      continue;

    int other_curr = agentPosition(pathfinders[aj], next_timestep - 1);
    int other_next = agentPosition(pathfinders[aj], next_timestep);
    bool other_at_goal = agentStaysAtGoal(pathfinders[aj], next_timestep);

    if(next_id == other_next) {
      if(excl_at_goal || other_at_goal) {
        score += lookup(conflict::target(next_timestep, excl, aj, next_id, -1));
        score += lookup(conflict::target(next_timestep, aj, excl, next_id, -1));
      } else {
        score += lookup(conflict(next_timestep, excl, aj, next_id, -1));
      }
    }

    if(next_timestep > 0 && curr_id == other_next && next_id == other_curr) {
      score += lookup(conflict(next_timestep - 1, excl, aj, next_id, curr_id));
    }
  }

  return score;
}

inline void clear_map(MAPF_Solver* s, geas::vec<int>& map, int t) {
  for(Agent_PF* p : s->pathfinders) {
    map[agentPosition(p, t)] = -1;
  }
}

int MAPF_Solver::monotoneSubchainStart(int dy, int dx, int ai, int t) const {
  int p(agentPosition(pathfinders[ai], t));
  for(--t; t >= 0; --t) {
    int q(agentPosition(pathfinders[ai], t));
    if(p == q)
      return t+1;
    if(col_of(p) != col_of(q) && col_of(p) - col_of(q) != dx)
      return t+1;
    if(row_of(p) != row_of(q) && row_of(p) - row_of(q) != dy)
      return t+1;
    p = q; 
  }
  return 0;
}

int MAPF_Solver::monotoneSubchainEnd(int dy, int dx, int ai, int t) const {
  int p(agentPosition(pathfinders[ai], t));
  int tMax(pathfinders[ai]->getPath().size());
  for(++t; t < tMax; ++t) {
    int q(agentPosition(pathfinders[ai], t));
    if(p == q)
      return t-1;
    if(col_of(p) != col_of(q) && col_of(q) - col_of(p) != dx)
      return t-1;
    if(row_of(p) != row_of(q) && row_of(q) - row_of(p) != dy)
      return t-1;
    p = q; 
  }
  return tMax-1;
}

bool MAPF_Solver::resolveConflicts(void) {
  int pMax = maxPathLength();
  
  geas::vec<bool> conflicting(pathfinders.size(), false);

  // First mark the occupied locations at t=0, then scan forward in time and
  // identify agents whose current paths collide in the same cell or swap cells.
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    int loc = pathfinders[ai]->getPath()[0];
    assert(nmap[loc] < 0); // Shouldn't be any conflicts at t0.
    nmap[loc] = ai;
  }
  // Run through the candidate plan, identify agents with conflicts.
  for(int t = 1; t < pMax; ++t) {
    ::std::swap(cmap, nmap);

    for(int ai = 0; ai < pathfinders.size(); ++ai) {
      int loc = agentPosition(pathfinders[ai], t);
      if(nmap[loc] >= 0) {
        // Edge conflict between agents ai and nmap[loc].
        conflicting[ai] = true;
        conflicting[nmap[loc]] = true;
      } else {
        nmap[loc] = ai;
      }
      if(cmap[loc] > 0 && cmap[loc] != ai) {
        // Get the new location of the agent we're replacing.
        int rloc = agentPosition(pathfinders[cmap[loc]], t);
        if(cmap[rloc] == ai) {
          // Edge conflict between agents ai and cmap[loc].
          conflicting[ai] = true;
          conflicting[cmap[loc]] = true; 
        }
      }
    }
    // Now we zero out the previous cmap.
    clear_map(this, cmap, t-1);
  }
  clear_map(this, nmap, pMax-1);
  
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    if(conflicting[ai]) {
      // Try re-routing agent ai, in the hopes of getting a better plan.
      pathfinders[ai]->find_bypass();
    }
  }
  return !checkForConflicts();
}

// Multiple-conflict handling
bool MAPF_Solver::checkForConflicts(void) {
  int pMax = maxPathLength();
  
  // Build the occupancy map one timestep at a time. Any collision discovered
  // here is converted into a high-level conflict object so it can be encoded
  // later by addConflict().
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    int loc = pathfinders[ai]->getPath()[0];
    assert(nmap[loc] < 0); // Shouldn't be any conflicts at t0.
    nmap[loc] = ai;
  }
  // All agents are interesting.
  agent_set.sz = pathfinders.size();

  for(int t = 1; t < pMax; ++t) {
    ::std::swap(cmap, nmap);

    // for(int ai = 0; ai < pathfinders.size(); ++ai) {
    for(int ai : agent_set.rev()) {
      int loc = agentPosition(pathfinders[ai], t);
      if(nmap[loc] >= 0) {
        // Already occupied.
        int aj(nmap[loc]);
        if(agentStaysAtGoal(pathfinders[ai], t) || agentStaysAtGoal(pathfinders[aj], t)) {
          if(!agentStaysAtGoal(pathfinders[ai], t))
            ::std::swap(ai, aj);
          conflict c(conflict::target(t, ai, aj, loc, -1));
          record_conflict(*this, c);
          new_conflicts.push(c);

          clear_map(this, cmap, t-1);
          clear_map(this, nmap, t);
          agent_set.remove(ai);
          continue;
        }
        int dy1 = row_of(agentPosition(pathfinders[ai], t)) - row_of(agentPosition(pathfinders[ai], t-1));
        int dx1 = col_of(agentPosition(pathfinders[ai], t)) - col_of(agentPosition(pathfinders[ai], t-1));
        int dy2 = row_of(agentPosition(pathfinders[aj], t)) - row_of(agentPosition(pathfinders[aj], t-1));
        int dx2 = col_of(agentPosition(pathfinders[aj], t)) - col_of(agentPosition(pathfinders[aj], t-1));
#ifdef MAPF_NO_RECTANGLES
        goto fallback;
#endif
        if(dx1 != dx2 && dy1 != dy2) {
          // This is a rectangle conflict
          int dy(dy1 + dy2);
          int dx(dx1 + dx2);
          
          // Make sure ai is the horizontal agent.
          if(dx2)
            ::std::swap(ai, aj);

          // Find the start positions
          int stH(monotoneSubchainStart(dy, dx, ai, t));
          int stV(monotoneSubchainStart(dy, dx, aj, t));
          
          int sH(agentPosition(pathfinders[ai], stH));
          int sV(agentPosition(pathfinders[aj], stV));

          // If there is overhang, adjust the locations.
          while(true) {
            if(dx * (col_of(sV) - col_of(sH)) < 0) {
              ++stV;     
              sV = agentPosition(pathfinders[aj], stV);
              continue;
            }
            if(dy * (row_of(sH) - row_of(sV)) < 0) {
              ++stH;
              sH = agentPosition(pathfinders[ai], stH); 
              continue;
            }
            break;
          }

          int etH(monotoneSubchainEnd(dy, dx, ai, t));
          int etV(monotoneSubchainEnd(dy, dx, aj, t));

          int eH(agentPosition(pathfinders[ai], etH));
          int eV(agentPosition(pathfinders[aj], etV));

          while(true) {
            if(dx * (col_of(eH) - col_of(eV)) < 0) {
              --etV;
              eV = agentPosition(pathfinders[aj], etV);
              continue;
            }
            if(dy * (row_of(eV) - row_of(eH)) < 0) {
              --etH;
              eH = agentPosition(pathfinders[ai], etH);
              continue;
            }
            break;
          }
          assert(stH <= t);
          assert(stV <= t);
          assert(t <= etH);
          assert(t <= etV);
          assert(dy * (row_of(sH) - row_of(sV)) >= 0);
          assert(dx * (col_of(sV) - col_of(sH)) >= 0);
          assert(dy * (row_of(eV) - row_of(eH)) >= 0);
          assert(dx * (col_of(eH) - col_of(eV)) >= 0);
           
          int locS(ml->linearize_coordinate(row_of(sV), col_of(sH)));
          int locE(ml->linearize_coordinate(row_of(eV), col_of(eH)));
          int t0(stH - abs(row_of(sH) - row_of(locS)));
          assert(t0 == stV - abs(col_of(sV) - col_of(locS)));
          conflict c(conflict::barrier(t0, ai, aj, locS, locE));
          record_conflict(*this, c);
          new_conflicts.push(c);
        } else {
#ifdef MAPF_NO_RECTANGLES
        fallback:
#endif
          conflict c(t, ai, nmap[loc], loc, -1);
          record_conflict(*this, c);
          new_conflicts.push(c);
        }

        clear_map(this, cmap, t-1);
        clear_map(this, nmap, t);
        agent_set.remove(ai);
        // return true;
        continue;
      }
      nmap[loc] = ai;
      if(cmap[loc] > 0 && cmap[loc] != ai) {
        // Get the new location of the agent we're replacing.
        int rloc = agentPosition(pathfinders[cmap[loc]], t);
        if(cmap[rloc] == ai) {
          // Edge conflict
          conflict c(t-1, ai, cmap[loc], loc, rloc);
          record_conflict(*this, c);
          new_conflicts.push(c);
          clear_map(this, cmap, t-1);
          clear_map(this, nmap, t);
          agent_set.remove(ai);
          // return true;
          continue;
        }
      }
    }
    // Now we zero out the previous cmap.
    clear_map(this, cmap, t-1);
  }
  clear_map(this, nmap, pMax-1);

  return new_conflicts.size() > 0;
}

//enum BarrierDir { UP = 0, LEFT = 1, DOWN = 2, RIGHT = 3 };
static int barrier_dx[4] = { 0, -1, 0, 1 };
static int barrier_dy[4] = { -1, 0, 1, 0 };

geas::patom_t MAPF_Solver::getBarrier(int ai, BarrierDir dir, int t, int p, int dur) {
  // If this barrier is trivially violated, just return F.
  int delta = ml->cols * barrier_dy[dir] + barrier_dx[dir];
  assert(t >= 0);
  if(t == 0 && p == pathfinders[ai]->engine.start_location)
    return geas::at_False;

  // Project the direction back in the appropriate direction, to see what set of barriers we're in.
  int p_ident;
  int t0;
  switch(dir) {
    case UP:
      p_ident = col_of(p);  
      t0 = t - row_of(p);
      break;
    case LEFT:
      p_ident = row_of(p);
      t0 = t - col_of(p);
      break;
    case DOWN:
      p_ident = col_of(p);
      t0 = t - row_of(p);
      break;
    case RIGHT:
    default:
      p_ident = row_of(p);
      t0 = t - col_of(p);
      break;
  }
  barrier_key k { ai, dir, p_ident, t0 };
  auto it(barrier_map.find(k));
  int idx;
  if(it != barrier_map.end()) {
    idx = (*it).second;
  } else {
    // Reuse identical barrier families across conflicts so we only allocate a
    // new GEAS atom the first time we see a given geometric template.
    idx = barriers.size();
    barriers.push();
    barrier_map.insert(::std::make_pair(k, idx));
  }
  
  // Check if this barrier has already been generated.
  geas::vec<barrier_data>& candidates(barriers[idx]);
  for(const barrier_data& b : candidates) {
    if(b.start == t && b.duration == dur) {
      // fprintf(stderr, "%%%% HIT!\n");
      return b.act;
    }
  }
  // If not, we'll create it. 
  geas::patom_t act(s.new_boolvar());
  // Set up entailment relationships.
#ifdef BARRIER_ENTAIL
  int end = t + dur;
  for(const barrier_data& b : candidates) {
    if(b.start <= t && end <= b.start + b.duration) {
      // b subsumes this.
      // fprintf(stderr, "%% Found super-barrier\n");
      add_clause(s.data, ~b.act, act);
    }
    if(t <= b.start && b.start + b.duration <= end) {
      // fprintf(stderr, "%% Found sub-barrier\n");
      add_clause(s.data, act, ~b.act);
    }
  }
#endif
  pathfinders[ai]->register_barrier(act, t, p, delta, dur); 
  candidates.push(barrier_data { act, t, dur });
  return act;
}

geas::patom_t MAPF_Solver::getTargetBarrier(int ai, int t, int p, int dur) {
  assert(t >= 0);
  if(dur <= 0)
    return geas::at_False;
  if(t == 0 && p == pathfinders[ai]->engine.start_location)
    return geas::at_False;

  geas::patom_t act(s.new_boolvar());
  pathfinders[ai]->register_target_barrier(act, t, p, dur);
  return act;
}

bool MAPF_Solver::addConflict(void) {
  HL_conflicts++;
  // Convert each detected high-level conflict into a GEAS constraint. The
  // solver keeps per-pattern caches so repeated conflicts reuse the same
  // symbolic variables where possible.
  for(auto new_conflict : new_conflicts) {
	    if(new_conflict.type == C_BARRIER) {
	      int aH(new_conflict.a1);
	      int aV(new_conflict.a2);
	      int p_s(new_conflict.b.s_loc);
	      int p_e(new_conflict.b.e_loc);

	      tracef("addConflict: rectangle t=%d agents=(%d,%d) corners=(%d,%d)->(%d,%d)",
	        new_conflict.timestamp, aH, aV, row_of(p_s), col_of(p_s), row_of(p_e), col_of(p_e));

      geas::vec<geas::clause_elt> barrier_atoms;

      // Entry barrier starts at top-left
      int s_time(new_conflict.timestamp);
      // fprintf(stderr, "%% Adding rectangle [%d] (%d, %d) -> (%d, %d)\n", s_time, row_of(p_s), col_of(p_s), row_of(p_e), col_of(p_e));
      int dt(::std::min(0, s_time));
      int h_dur(1 + abs(row_of(p_e) - row_of(p_s)));
      int h_delta(row_of(p_s) < row_of(p_e) ? ml->cols : -ml->cols);
      
	      BarrierDir dH(row_of(p_s) < row_of(p_e) ? DOWN : UP);
	      if(s_time > 0 || pathfinders[aH]->engine.start_location != p_s - dt*h_delta) {
	        tracef("addConflict: rectangle horizontal barrier agent=%d start_t=%d start=(%d,%d) dur=%d delta=%d",
	          aH, s_time - dt, row_of(p_s), col_of(p_s), h_dur + dt, h_delta);
	        barrier_atoms.push(getBarrier(aH, dH, s_time - dt, p_s - dt*h_delta, h_dur + dt));
	      }

	      int eh_start(ml->linearize_coordinate(row_of(p_s), col_of(p_e)));
	      int eh_time(s_time + abs(col_of(p_e) - col_of(p_s)));
	      tracef("addConflict: rectangle horizontal barrier agent=%d start_t=%d start=(%d,%d) dur=%d delta=%d",
	        aH, eh_time - dt, row_of(p_s), col_of(p_e), h_dur + dt, h_delta);
	      barrier_atoms.push(getBarrier(aH, dH, eh_time - dt, eh_start - dt*h_delta, h_dur+dt));

	      int v_dur(1 + abs(col_of(p_e) - col_of(p_s)));
	      int v_delta(col_of(p_s) < col_of(p_e) ? 1 : -1);
	      BarrierDir dV(col_of(p_s) < col_of(p_e) ? RIGHT : LEFT);

	      if(s_time > 0 || pathfinders[aV]->engine.start_location != p_s - dt*v_delta) {
	        tracef("addConflict: rectangle vertical barrier agent=%d start_t=%d start=(%d,%d) dur=%d delta=%d",
	          aV, s_time - dt, row_of(p_s), col_of(p_s), v_dur+dt, v_delta);
	        barrier_atoms.push(getBarrier(aV, dV, s_time - dt, p_s - dt*v_delta, v_dur+dt));
	      }

	      int ev_start(ml->linearize_coordinate(row_of(p_e), col_of(p_s)));
	      int ev_time(s_time + abs(row_of(p_e) - row_of(p_s)));
	      tracef("addConflict: rectangle vertical barrier agent=%d start_t=%d start=(%d,%d) dur=%d delta=%d",
	        aV, ev_time - dt, row_of(p_e), col_of(p_s), v_dur+dt, v_delta);
	      barrier_atoms.push(getBarrier(aV, dV, ev_time - dt, ev_start - dt*v_delta, v_dur+dt));

      // One of the barriers must be active
      add_clause(*s.data, barrier_atoms);
      tracef("addConflict: rectangle encoded as %d barrier atom(s)", (int) barrier_atoms.size());
    } else if(new_conflict.type == C_TARGET) {
      int target_agent(new_conflict.a1);
      int moving_agent(new_conflict.a2);
      int target_loc(new_conflict.p.loc1);
      tracef("addConflict: target t=%d target agent=%d moving agent=%d loc=%d",
        new_conflict.timestamp, target_agent, moving_agent, target_loc);

      target_key k { new_conflict.timestamp, target_agent, moving_agent, target_loc };
      auto it(target_map.find(k));
      int idx;
      if(it != target_map.end()) {
        idx = (*it).second;
      } else {
        idx = target_constraints.size();
        target_map.insert(::std::make_pair(k, idx));
        target_constraints.push(target_data { s.new_boolvar(), false });
      }

      target_data& c(target_constraints[idx]);
      if(!c.attached) {
        int dur = ::std::max(1, cost_ub - new_conflict.timestamp + 1);
        geas::patom_t lock_at(getTargetBarrier(moving_agent, new_conflict.timestamp, target_loc, dur));
        if(enable_target_symmetry) {
          // Target symmetry split:
          // 1) forbid the stationary agent from using the target at time t-1
          // 2) forbid the moving agent from using the target cell from that time onward
          geas::patom_t stationary_lock(
            getTargetBarrier(target_agent, new_conflict.timestamp - 1, target_loc, dur));
          add_clause(s.data, ~c.sel, stationary_lock);
          add_clause(s.data, c.sel, lock_at);
        } else {
          // Fall back to a direct target lock on the moving agent only.
          c.sel = lock_at;
        }
        c.attached = true;
      }
    } else {
      int loc1 = new_conflict.p.loc1;
      int loc2 = new_conflict.p.loc2;
      if(loc2 > loc1)
        ::std::swap(loc1, loc2);

      tracef("addConflict: mutex t=%d agents=(%d,%d) locs=(%d,%d)", new_conflict.timestamp, new_conflict.a1, new_conflict.a2, loc1, loc2);
        
      cons_key k { new_conflict.timestamp, loc1, loc2 };
      auto it(cons_map.find(k));
       
      int idx;
      if(it != cons_map.end()) {
        idx = (*it).second;
      } else {
        idx = constraints.size();
        cons_map.insert(::std::make_pair(k, idx));
        constraints.push(cons_data { s.new_intvar(0, pathfinders.size()-1), btset::bitset(pathfinders.size()) });
      }

      int a1(new_conflict.a1);
      int a2(new_conflict.a2);
      cons_data& c(constraints[idx]);
      if(!c.attached.elem(a1)) {
        patom_t at(c.sel != a1);
        while(s.level() > 0 && at.lb(s.data->ctx()))
          s.backtrack();
        pathfinders[a1]->register_obstacle(at, k.timestamp, k.loc1, k.loc2);
        c.attached.insert(a1);
      }
      if(!c.attached.elem(a2)) {
        patom_t at(c.sel != a2);
        while(s.level() > 0 && at.lb(s.data->ctx()))
          s.backtrack();
        pathfinders[a2]->register_obstacle(at, k.timestamp, k.loc1, k.loc2);
        c.attached.insert(a2);
      }
      // FIXME: Abstract properly
      //s.data->confl.pred_saved[c.sel.p>>1].val = geas::from_int((rand() % 2 ? a1 : a2));
    }
    if(super_verbose) {
      fprintf(stderr, "MAPF paths after new conflict:\n");
      printPaths(stderr);
    }
  }
  new_conflicts.clear();
  return true;
}

bool MAPF_Solver::processCore(geas::vec<geas::patom_t>& core) {
  // If the core is empty, we're unsatisfiable.
  // Shouldn't usually happen, since everyone can just wait at the
  // start state.
  if(core.size() == 0)
    return false;

  // Translate the unsat core into a tighter family of lower bounds. The
  // resulting penalties are fed back into the next solve attempt.
  geas::vec<int> idxs;
  uint64_t Dmin = UINT64_MAX;

  for(geas::patom_t c : core) {
    // Every assumption should be in the table.
    int c_idx = (*penalty_table.find(c.pid)).second;
    assert(c.val > penalties[c_idx].lb);
    uint64_t c_delta = c.val - penalties[c_idx].lb;
    Dmin = ::std::min(Dmin, c_delta);
    idxs.push(c_idx);
  }
  // We can increase the lower bound by Dmin.
  // Introduce the new penalty terms, and increase the existing bounds by Dmin.
  geas::vec<int> coeffs(idxs.size(), 1);
  for(uint64_t d = 1; d <= Dmin; ++d) {
    geas::vec<geas::patom_t> slice;
    for(int ci : idxs) {
      slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
    }
    // New penalty var.
    intvar p(s.new_intvar(0, slice.size()-1));
    geas::bool_linear_ge(s.data, geas::at_True, p, coeffs, slice, -1);
    penalty_table.insert(::std::make_pair(p.p, penalties.size()));
    penalties.push(penalty { p.p, geas::from_int(0) });
  }

  //cost_lb += Dmin;
  for(int ci : idxs) {
    penalties[ci].lb += Dmin;
  }
  return Dmin;
}

bool MAPF_Solver::runUCIter(void) {
  // One GEAS solve step under the current assumptions. SAT means we have a
  // candidate plan; UNSAT means the current assumptions are inconsistent and
  // should be processed into a stronger lower bound.
  switch(s.solve()) {
    // No solution, we've got a new core.
    case geas::solver::UNSAT:
      return false;
    // Candidate optimal solution. Check for conflicts
    case geas::solver::SAT:  
      return true;
    case geas::solver::UNKNOWN:
      throw SolveAborted { };
  }
  return false;
}

MAPF_Solver::~MAPF_Solver(void) {

}

bool MAPF_MinCost(MAPF_Solver& mapf) {
  // Reset any existing assumptions
  mapf.s.clear_assumptions(); 

  geas::vec<MAPF_Solver::penalty> penalties;
  ::std::unordered_map<geas::pid_t, int> penalty_table;

  int cost_lb(0); 
  // Seed the optimization loop with each agent's current single-agent plan
  // cost, then keep tightening those bounds as GEAS produces cores.
  for(Agent_PF* p : mapf.pathfinders) {
    cost_lb += p->pathCost();
    geas::pid_t id(p->cost.p);
    penalty_table.insert(::std::make_pair(id, penalties.size()));
    penalties.push(MAPF_Solver::penalty { id, geas::from_int(p->pathCost()) });
  }
  mapf.tracef("MAPF_MinCost: initial lower bound %d", cost_lb);
#ifdef DEBUG_UC
  fprintf(stderr, "%%%% Initial bound: %d\n", cost_lb);
#endif
 
  geas::vec<geas::patom_t> assumps;
  for(MAPF_Solver::penalty& p : penalties)
    assumps.push(geas::le_atom(p.p, p.lb));

  geas::vec<geas::patom_t> core;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();
    if(core.size() == 0)
      return false;
    // Look at the core elements.
    geas::vec<int> idxs;
    uint64_t Dmin = UINT64_MAX;

    for(geas::patom_t c : core) {
      // Every assumption should be in the table.
      int c_idx = (*penalty_table.find(c.pid)).second;
      assert(c.val > penalties[c_idx].lb);
      uint64_t c_delta = c.val - penalties[c_idx].lb;
      Dmin = ::std::min(Dmin, c_delta);
      idxs.push(c_idx);
    }
    // We can increase the lower bound by Dmin.
    // Introduce the new penalty terms, and increase the existing bounds by Dmin.
    geas::vec<int> coeffs(idxs.size(), 1);
    for(uint64_t d = 1; d <= Dmin; ++d) {
      geas::vec<geas::patom_t> slice;
      for(int ci : idxs) {
        slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
      }
      // New penalty var.
      intvar p(mapf.s.new_intvar(0, slice.size()-1));
      geas::bool_linear_ge(mapf.s.data, geas::at_True, p, coeffs, slice, -1);
      penalty_table.insert(::std::make_pair(p.p, penalties.size()));
      penalties.push(MAPF_Solver::penalty { p.p, geas::from_int(0) });
    }
    // Update the existing penalties and bounds
    for(int ci : idxs) {
      penalties[ci].lb += Dmin;
    }
    cost_lb += Dmin;
    mapf.cost_lb = cost_lb;
    mapf.tracef("MAPF_MinCost: core size %d -> lower bound %d", (int) core.size(), cost_lb);
#ifdef DEBUG_UC
      fprintf(stderr, "%%%% Found core of size (%d), current lower bound %d\n", core.size(), cost_lb);
#endif
    
    // Now set up the updated assumptions
    assumps.clear();
    for(MAPF_Solver::penalty& p : penalties)
      assumps.push(geas::le_atom(p.p, p.lb));
  }
  return true;
}

bool MAPF_MinMakespan(MAPF_Solver& mapf) {
  mapf.s.clear_assumptions();
  int makespan_lb(0);
  ::std::unordered_map<geas::pid_t, unsigned int> agent_table;
  unsigned int ai = 0;
  for(Agent_PF* p : mapf.pathfinders) {
    makespan_lb = ::std::max(makespan_lb, (int) p->pathCost());
    agent_table.insert(::std::make_pair(p->cost.p, ai));
    ++ai;
  }

  geas::vec<geas::patom_t> assumps;

  for(Agent_PF* p : mapf.pathfinders)
    assumps.push(p->cost <= makespan_lb);
  
  geas::vec<geas::patom_t> core;
  mapf.cost_lb = makespan_lb;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();
    // Globally infeasible (somehow, probably should be unreachable).
    if(core.size() == 0)
      return false;
    // Otherwise, extract the new makespan from the core. 
    int new_makespan(0);
    for(geas::patom_t at : core) {
      // Turn the core atoms back into bounds.
      auto it(agent_table.find(at.pid));
      assert(it != agent_table.end());
      const geas::intvar& x(mapf.pathfinders[(*it).second]->cost);
      new_makespan = ::std::max(new_makespan, x.lb_of_pval(at.val));
    }
    assert(new_makespan > makespan_lb);
    makespan_lb = mapf.cost_lb = new_makespan;
    assumps.clear();
    for(Agent_PF* p : mapf.pathfinders)
      assumps.push(p->cost <= makespan_lb);
  }

  return true;
}

bool MAPF_MaxDeadlines(MAPF_Solver& mapf, geas::vec<unsigned int>& deadlines) {
  assert(mapf.pathfinders.size() == deadlines.size());
  mapf.s.clear_assumptions();
  int cost_lb(0);
  unsigned int ai = 0;
  // In the result, even values are thresholds,
  // odd values are counts.
  ::std::unordered_map<geas::pid_t, int> penalty_table;
  geas::vec<bool> enforced(deadlines.size(), true);
  geas::vec<MAPF_Solver::penalty> penalties;

  for(Agent_PF* p : mapf.pathfinders) {
    // If the deadline is infeasible, just add it to the set of penalties.
    if(p->pathCost() > deadlines[ai]) {
      cost_lb++;
      enforced[ai] = false;
    } else {
      geas::pid_t id(p->cost.p);
      penalty_table.insert(::std::make_pair(id, ai<<1));
    }
  }

  // Initially, we only have thresholds.
  geas::vec<geas::patom_t> assumps;
  for(int ai : geas::irange(mapf.pathfinders.size())) {
    if(enforced[ai]) {
      const intvar& x(mapf.pathfinders[ai]->cost);
      assumps.push(x <= deadlines[ai]);
    }
  }

  geas::vec<geas::patom_t> core;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();  
    if(core.size() == 0)
      return false;
    // Look at the core elements.
    geas::vec<int> idxs;
    uint64_t Dmin = UINT64_MAX;

    for(geas::patom_t c : core) {
      // Every assumption should be in the table.
      int c_tag = (*penalty_table.find(c.pid)).second;
      idxs.push(c_tag);
      if(!(c_tag & 1)) { // First bit is zero; original deadline.
        int ai(c_tag>>1);  
        // If a deadline is in the core, we can relax by at most one step. 
        Dmin = 1;
        enforced[ai] = 0;
      } else {
        int p_idx(c_tag>>1);
        assert(c.val > penalties[p_idx].lb);
        uint64_t c_delta = c.val - penalties[p_idx].lb;
        Dmin = ::std::min(Dmin, c_delta);
      }
    }
    // We can increase the lower bound by Dmin.
    // Introduce the new penalty terms, and increase the existing bounds by Dmin.
    geas::vec<int> coeffs(idxs.size(), 1);
    for(uint64_t d = 1; d <= Dmin; ++d) {
      geas::vec<geas::patom_t> slice;
      for(int c_tag : idxs) {
        if(!(c_tag & 1)) {
          int ai(c_tag>>1);
          slice.push(mapf.pathfinders[ai]->cost > deadlines[ai]); 
        } else {
          int ci(c_tag>>1);
          slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
          penalties[ci].lb += Dmin;
        }
      }
      // New penalty var.
      intvar p(mapf.s.new_intvar(0, slice.size()-1));
      geas::bool_linear_ge(mapf.s.data, geas::at_True, p, coeffs, slice, -1);
      penalty_table.insert(::std::make_pair(p.p, penalties.size()));
      penalties.push(MAPF_Solver::penalty { p.p, geas::from_int(0) });
    }
    // Update the total cost.
    cost_lb += Dmin;
    
    // Now set up the updated assumptions
    assumps.clear();
    int ai = 0;
    for(Agent_PF* p : mapf.pathfinders) {
      if(enforced[ai])
        assumps.push(p->cost <= deadlines[ai]);
      ++ai;
    }
    for(MAPF_Solver::penalty& p : penalties)
      assumps.push(geas::le_atom(p.p, p.lb)); 
  }
  return true;
}

}
#endif
