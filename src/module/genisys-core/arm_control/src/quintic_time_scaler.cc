#include "quintic_time_scaler.h"
#include <cmath>

bool QuinticTimeScaler::Plan(const PlanConfig& config) {
  if (config.T <= 0.0) { return false; }
  duration_ = config.T;
  a0_ = config.s0;
  a1_ = 0.0;
  a2_ = 0.0;
  a3_ = (20.0 * (config.s1 - config.s0)) / (2.0 * std::pow(config.T, 3));
  a4_ = (30.0 * (config.s0 - config.s1)) / (2.0 * std::pow(config.T, 4));
  a5_ = (12.0 * (config.s1 - config.s0)) / (2.0 * std::pow(config.T, 5));
  return true;
}

bool QuinticTimeScaler::Evaluate(double t, TimeScalerBase::State& state) const {
  if (t < 0.0 || t > duration_) { return false; }
  state.t = t;
  state.s = a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) + a4_ * std::pow(t, 4) + a5_ * std::pow(t, 5);
  state.sd = a1_ + 2.0 * a2_ * t + 3.0 * a3_ * std::pow(t, 2) + 4.0 * a4_ * std::pow(t, 3) + 5.0 * a5_ * std::pow(t, 4);
  state.sdd = 2.0 * a2_ + 6.0 * a3_ * t + 12.0 * a4_ * std::pow(t, 2) + 20.0 * a5_ * std::pow(t, 3);
  return true;
}

double QuinticTimeScaler::GetDuration() const
{
    return duration_;
}
