#include <yaml-cpp/yaml.h>

#include <lazycbs/map_loader.h>
#include <lazycbs/agents_loader.h>
#include <lazycbs/egraph_reader.h>
#include <lazycbs/mapf-solver.h>

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <boost/program_options.hpp>

// This flag controls early solver termination.
volatile static std::sig_atomic_t terminated = 0;

void catch_int (int sig) {
  (void)sig;
  terminated = 1;
}
void set_handlers(void) {
  std::signal(SIGINT, catch_int);
}
void clear_handlers(void) {
  std::signal(SIGINT, SIG_DFL);
}

// Serialize the plan in the YAML schedule format expected by the example.
void write_schedule(std::ostream& out,
                    const std::vector<std::vector<std::pair<int, int>>>& paths) {
  for (size_t agent = 0; agent < paths.size(); ++agent) {
    out << "  agent" << agent << ":" << std::endl;
    const std::vector<std::pair<int, int>>& path = paths[agent];
    for (size_t t = 0; t < path.size(); ++t) {
      out << "    - x: " << path[t].first << std::endl
          << "      y: " << path[t].second << std::endl
          << "      t: " << t << std::endl;
    }
  }
}


int main(int argc, char* argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");

#ifndef LAZYCBS_DEFAULT_TARGET_SYMMETRY
#define LAZYCBS_DEFAULT_TARGET_SYMMETRY 1
#endif

  std::string inputFile;
  std::string outputFile = "solver_output.yaml";
  bool verbose = false;
  bool super_verbose = false;
  bool target_symmetry = LAZYCBS_DEFAULT_TARGET_SYMMETRY != 0;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")(
      "output,o", po::value<std::string>(&outputFile),
      "output file (YAML)")(
      "verbose,v", "enable MAPF solver logging")(
      "super-verbose", "enable MAPF solver logging and print agent paths")(
      "no-target-symmetry", "disable the target-symmetry split");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }

    po::notify(vm);
    verbose = vm.count("verbose") != 0u;
    super_verbose = vm.count("super-verbose") != 0u;
    if (vm.count("no-target-symmetry") != 0u)
      target_symmetry = false;
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::vector<std::pair<int, int>> obstacles;
  std::vector<std::pair<int, int>> goals;
  std::vector<std::pair<int, int>> starts;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.emplace_back(node[0].as<int>(), node[1].as<int>());
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(start[0].as<int>(), start[1].as<int>());
    goals.emplace_back(goal[0].as<int>(), goal[1].as<int>());
  }

  std::pair<int, std::vector<std::vector<std::pair<int, int>>>> solution;

  set_handlers();
  lazycbs::MapLoader ml(dimx, dimy, obstacles);
  lazycbs::AgentsLoader al(starts, goals);
  lazycbs::EgraphReader egr;
  lazycbs::MAPF_Solver mapf1(ml, al, egr, 1e8, verbose, super_verbose, target_symmetry);

  clear_handlers();

  auto lazycbs_start = std::chrono::system_clock::now();
  bool success = lazycbs::MAPF_MinCost(mapf1);
  if (success)
    mapf1.getPaths(&solution);
  std::cout << "Cost :: " << solution.first << std::endl;
  auto lazycbs_end = std::chrono::system_clock::now();
  auto lazycbs_time = std::chrono::duration<double>(lazycbs_end - lazycbs_start).count();
  if (success) {
    std::cout << "Planning successful! " << std::endl;
    if (verbose || super_verbose) {
      std::cerr << "MAPF stats: ";
      mapf1.printStats(stderr);
      std::cerr << std::endl;
    }
    if (super_verbose) {
      std::cerr << "MAPF agent paths:" << std::endl;
      mapf1.printPaths(stderr);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << solution.first << std::endl;
    out << "  runtime: " << lazycbs_time << std::endl;
    out << "schedule:" << std::endl;
    write_schedule(out, solution.second);
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  std::cout << "TIME TAKEN TO COMPLETE THE TASK ::" << std::endl
            << "LAZYCBS :: " << lazycbs_time << std::endl << std::endl << std::endl;


  return 0;
}
