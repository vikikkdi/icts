#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <icts/icts.hpp>
#include <icts/ICTS_.hpp>
#include "mapf.hpp"

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <chrono>

using namespace ICT;

int main(int argc, char* argv[]) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");

  std::string inputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::vector<std::pair<int, int> > obstacles;
  std::vector<std::pair<int, int> > goals;
  std::vector<std::pair<int, int> > starts;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.emplace_back(std::make_pair(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));
  }

  mapf_adapters::mapf mapf(dimx, dimy, obstacles, goals);
  
  std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution;
  ICT::ICTS<mapf_adapters::mapf> mapf_icts;
  ICT_NEW::ICTS<mapf_adapters::mapf> mapf_icts_;

  auto icts_start = std::chrono::system_clock::now();
  bool success = mapf_icts_.search(mapf, starts, &solution);
  std::cout<<"Cost :: "<<solution.first<<std::endl;
  auto icts_end = std::chrono::system_clock::now();
  auto icts_time = std::chrono::duration<double>(icts_end - icts_start).count();

  if (success) {
    std::cout << "Planning successful! " << std::endl;

    std::ofstream out("../example/output_icts.yaml");
    out << "statistics:" << std::endl;
    out << "  cost: " << solution.first << std::endl;
    out << "  runtime: " << icts_time << std::endl;
    out << "schedule:" << std::endl;

    int count = 0;

    for(auto it=solution.second.begin(); it!=solution.second.end();++it){
      out << "  agent" << count << ":" << std::endl;
      std::vector<std::pair<int, int> > output;
      output = *it;
      for(int i=0; i<output.size(); i++){
        out << "    - x: " << output[i].first << std::endl
          << "      y: " << output[i].second << std::endl
          << "      t: " << i << std::endl;
      }
      count++;
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  std::cout<<"TIME TAKEN TO COMPLETE THE TASK ::"<<std::endl
            <<"ICTS :: "<<icts_time<<std::endl<<std::endl<<std::endl;



  return 0;
}