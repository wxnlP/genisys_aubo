#pragma once

#include "time_scaler_base.h"

class LSPBTimeScaler : public TimeScalerBase {
public:
    LSPBTimeScaler() = default;
    ~LSPBTimeScaler() override = default;

    bool Plan(const PlanConfig& config) override;
    bool Evaluate(double t, State& state) const override;
    double GetDuration() const override;
private:
    double duration_{0.0};
    double tb_{0.0};
    double a_{0.0};
    double v_{0.0};
};