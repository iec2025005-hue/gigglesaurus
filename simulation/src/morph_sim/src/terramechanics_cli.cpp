// Prints the wheel-soil state for one operating point so tests can cross-check C++ against Python.
// usage: terramechanics_cli normal thrust radius width kc kphi n cohesion phi K limit reload gain drag sliplimit rut
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include "terramechanics_model.hpp"
int main(int argc, char **argv) {
  if (argc != 17) {std::cerr << "expected 16 numeric arguments\n"; return 2;}
  double a[16]; for (int i = 0; i < 16; ++i) a[i] = std::atof(argv[i + 1]);
  morph_sim::terramechanics::Soil soil{a[4], a[5], a[6], a[7], a[8], a[9]};
  morph_sim::terramechanics::Config config{a[10], a[11], a[12], a[13], a[14]};
  const auto s = morph_sim::terramechanics::WheelSoilState(a[0], a[1], a[2], a[3], soil, config, a[15]);
  std::cout << std::setprecision(15) << "{\"static_sinkage_m\":" << s.staticSinkage << ",\"sinkage_m\":" << s.sinkage << ",\"slip_ratio\":" << s.slip
            << ",\"contact_length_m\":" << s.contactLength << ",\"thrust_limit_N\":" << s.thrustLimit << ",\"compaction_resistance_N\":" << s.compaction
            << ",\"slip_resistance_N\":" << s.slipResistance << ",\"resistance_N\":" << s.resistance << ",\"rut_depth_m\":" << s.rutDepth << "}\n";
  return 0;
}
