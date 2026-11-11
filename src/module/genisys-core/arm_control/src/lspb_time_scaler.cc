#include "lspb_time_scaler.h"

#include <cmath>

bool LSPBTimeScaler::Plan(const PlanConfig& config) {
  duration_ = config.T;
  a_ = config.a_max;
  v_ = config.v_max;

  if (duration_ <= 0.0) {
      // 使用v和a计算T
      // 若 v²/a >= 1，无法形成梯形剖面，改为三角形剖面：自动降低峰值速度
      if (std::pow(v_, 2) / a_ >= 1) {
        // 三角形剖面：v_peak = sqrt(a)，tb = sqrt(1/a)，duration = 2*tb
        v_ = std::sqrt(a_) * (1.0 - 1e-9);
      }
      duration_ = (a_ + std::pow(v_, 2)) / (v_ * a_);
  } else if (duration_ > 0.0) {
      // 使用T和v计算a
      if (duration_ * v_ <= 1 || duration_ * v_ > 2) {return false;}
      a_ = std::pow(v_, 2) / (duration_ * v_ - 1.0);
  }
  // if (std::pow(v_, 2) / a_ >= 1) {
  //   return false;
  // }
  // duration_ = (a_ + std::pow(v_, 2)) / (v_ * a_);
  tb_ = v_ / a_;
  return true;
}

bool LSPBTimeScaler::Evaluate(double t, State& state) const {
  state.t = t;
  if (t < 0) {
    state.s = 0.0;
    state.sd = 0.0;
    state.sdd = 0.0;
  } else if (t <= tb_) {
    state.s = 0.5 * a_ * std::pow(t, 2);
    state.sd = a_ * t;
    state.sdd = a_;
  } else if (t < duration_ - tb_) {
    state.s = v_ * (t - tb_ / 2);
    state.sd = v_;
    state.sdd = 0.0;
  } else if (t < duration_) {
    double td = duration_ - t;
    state.s = 1.0 - 0.5 * a_ * std::pow(td, 2);
    state.sd = a_ * td;
    state.sdd = -a_;
  } else {
    state.s = 1.0;
    state.sd = 0.0;
    state.sdd = 0.0;
  }
  return true;
}

double LSPBTimeScaler::GetDuration() const { return duration_; }