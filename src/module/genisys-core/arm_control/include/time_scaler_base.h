#pragma once

class TimeScalerBase {
public:
    struct State {
        double s;      // 归一化位置 [0, 1]
        double sd;     // 归一化速度 
        double sdd;    // 归一化加速度 
        double t;      // 当前时间
    };

    struct PlanConfig {
        double s0{0.0};
        double s1{1.0};
        double T{0.0};     // 如果为0，则参考 v_max
        double v_max{0.0};
        double a_max{0.0};
    };

    TimeScalerBase() = default;
    virtual ~TimeScalerBase() = default;

    virtual bool Plan(const PlanConfig& config) = 0;
    virtual bool Evaluate(double t, State& state) const = 0;
    virtual double GetDuration() const = 0;
};