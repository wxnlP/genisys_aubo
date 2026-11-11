#pragma once

#include "time_scaler_base.h"

class QuinticTimeScaler : public TimeScalerBase {
public:
    QuinticTimeScaler() = default; 
    ~QuinticTimeScaler() override = default;

    bool Plan(const PlanConfig& config) override;
    bool Evaluate(double t, State& state) const override;
    double GetDuration() const override;
private:
    double a0_{0.0};
    double a1_{0.0};
    double a2_{0.0};
    double a3_{0.0};
    double a4_{0.0};
    double a5_{0.0};
    double duration_{0.0};
};
