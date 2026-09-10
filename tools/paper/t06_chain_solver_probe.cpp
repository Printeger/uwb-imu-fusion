#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "uifgo/nlos_discovery.h"

namespace {

std::vector<double> ParseVector(const std::string& text) {
  std::vector<double> values;
  std::stringstream input(text);
  std::string token;
  while (std::getline(input, token, ',')) {
    if (token.empty()) throw std::invalid_argument("empty vector token");
    size_t used = 0;
    const double value = std::stod(token, &used);
    if (used != token.size()) throw std::invalid_argument("invalid number");
    values.push_back(value);
  }
  return values;
}

void WriteVector(const std::vector<double>& values) {
  std::cout << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << values[i];
  }
  std::cout << ']';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::vector<double> e, w;
    uifgo::FusedLassoOptions options;
    options.primal_absolute_tolerance_m = 1e-10;
    options.primal_relative_tolerance = 1e-8;
    options.dual_absolute_tolerance_objective_per_m = 1e-10;
    options.dual_relative_tolerance = 1e-8;
    options.kkt_tolerance_objective_per_m = 2e-7;
    options.tv_subgradient_tolerance_objective_per_m = 2e-7;
    options.max_iterations = 30000;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (i + 1 >= argc) throw std::invalid_argument(arg + " needs value");
      const std::string value = argv[++i];
      if (arg == "--e") e = ParseVector(value);
      else if (arg == "--weights") w = ParseVector(value);
      else if (arg == "--lambda-l1") options.lambda_l1 = std::stod(value);
      else if (arg == "--lambda-tv") options.lambda_tv = std::stod(value);
      else if (arg == "--rho-scale") options.rho_scale = std::stod(value);
      else throw std::invalid_argument("unknown argument " + arg);
    }
    const auto result =
        uifgo::SolveNonnegativeFusedLassoChain(e, w, options);
    std::cout << std::setprecision(17)
              << "{\"status\":\""
              << uifgo::FusedLassoStatusName(result.status)
              << "\",\"reason\":\"" << result.reason << "\",\"u\":";
    WriteVector(result.u);
    std::cout << ",\"p\":";
    WriteVector(result.p);
    std::cout << ",\"objective\":" << result.objective
              << ",\"primal_residual_m\":" << result.primal_residual_m
              << ",\"primal_threshold_m\":" << result.primal_threshold_m
              << ",\"dual_residual_objective_per_m\":"
              << result.dual_residual_objective_per_m
              << ",\"dual_threshold_objective_per_m\":"
              << result.dual_threshold_objective_per_m
              << ",\"max_kkt_violation_objective_per_m\":"
              << result.max_kkt_violation_objective_per_m
              << ",\"max_tv_subgradient_violation_objective_per_m\":"
              << result.max_tv_subgradient_violation_objective_per_m
              << ",\"rho_objective_per_m2\":"
              << result.rho_objective_per_m2
              << ",\"iterations\":" << result.iterations << "}\n";
    return result.converged() ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
