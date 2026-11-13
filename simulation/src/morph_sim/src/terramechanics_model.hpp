// Wheel-soil model for the V2 terramechanics emulation. Mirrors morph_sim/terramechanics.py;
// tests/test_terramechanics.py cross-checks the two through the terramechanics_cli executable.
// Force-level emulation over rigid contact: sinkage and soil slip are state variables only.
#ifndef MORPH_SIM_TERRAMECHANICS_MODEL_HPP
#define MORPH_SIM_TERRAMECHANICS_MODEL_HPP
#include <algorithm>
#include <cmath>

namespace morph_sim::terramechanics {

struct Soil {double kc{1e4}, kphi{1e6}, n{1.1}, cohesion{1e3}, phi{.5}, shearLength{.02};};
struct Config {double sinkageLimitFraction{.5}, reloadFraction{.1}, slipSinkageGain{0.}, slipDrag{0.}, slipLimit{.9};};
constexpr int kSolverIterations = 24;
constexpr double kSolverRelaxation = .5;
struct State {double staticSinkage{0.}, sinkage{0.}, slip{0.}, contactLength{0.}, thrustLimit{0.}, compaction{0.}, slipResistance{0.}, resistance{0.}, rutDepth{0.};};

// x >= 0 with 1 - (1 - exp(-x))/x = fraction; x = s l / K (Janosi-Hanamoto).
inline double SolveSlipParameter(double fraction) {
  fraction = std::clamp(fraction, 0., .98);
  if (fraction < 1e-9) return 0.;
  double low = 0., high = 1.;
  while (1. - (1. - std::exp(-high)) / high < fraction) high *= 2.;
  for (int i = 0; i < 60; ++i) {
    const double mid = .5 * (low + high);
    const double value = mid > 1e-12 ? 1. - (1. - std::exp(-mid)) / mid : 0.;
    if (value < fraction) low = mid; else high = mid;
  }
  return .5 * (low + high);
}

inline double VirginWork(double z, double width, const Soil &soil) {
  return (soil.kc + width * soil.kphi) * std::pow(std::max(0., z), soil.n + 1.) / (soil.n + 1.);
}

inline State WheelSoilState(double normal, double thrust, double radius, double width, const Soil &soil, const Config &config, double rutDepth = 0.) {
  State s; s.rutDepth = std::max(0., rutDepth);
  if (!(normal > 0.) || !std::isfinite(normal)) return s;
  const double modulus = soil.kc + width * soil.kphi, diameter = 2. * radius, limit = config.sinkageLimitFraction * radius;
  s.staticSinkage = std::min(limit, std::pow(3. * normal / ((3. - soil.n) * modulus * std::sqrt(diameter)), 2. / (2. * soil.n + 1.)));
  s.sinkage = s.staticSinkage; s.contactLength = std::sqrt(diameter * s.staticSinkage);
  for (int i = 0; i < kSolverIterations; ++i) {  // slip and slip sinkage solved together, damped
    s.contactLength = std::sqrt(diameter * s.sinkage);
    s.thrustLimit = width * s.contactLength * soil.cohesion + normal * std::tan(soil.phi);
    const double x = SolveSlipParameter(s.thrustLimit > 0. ? std::abs(thrust) / s.thrustLimit : 0.);
    const double target = s.contactLength > 0. ? std::min(config.slipLimit, x * soil.shearLength / s.contactLength) : 0.;
    s.slip += kSolverRelaxation * (target - s.slip);
    s.sinkage = std::min(limit, s.staticSinkage * (1. + config.slipSinkageGain * ((1. + s.slip) / (1. - .5 * s.slip) - 1.)));
  }
  const double shared = std::min(s.sinkage, s.rutDepth);
  s.compaction = VirginWork(s.sinkage, width, soil) - VirginWork(shared, width, soil) + config.reloadFraction * VirginWork(shared, width, soil);
  s.slipResistance = config.slipDrag >= .5 ? std::abs(thrust) * s.slip / (1. - s.slip) : 0.;
  s.resistance = s.compaction + s.slipResistance;
  s.rutDepth = std::max(s.sinkage, s.rutDepth);
  return s;
}

}  // namespace morph_sim::terramechanics
#endif
