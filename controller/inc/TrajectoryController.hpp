#ifndef CONTROLLER_INC_TRAJECTORYCONTROLLER_H
#define CONTROLLER_INC_TRAJECTORYCONTROLLER_H

#include "ReferencePath.hpp"
#include "Vehicle.hpp"
#include <algorithm>
#include <cmath>
#include <PIController.hpp>
#include <iostream>

class TrajectoryController
{
public:
    TrajectoryController() = default;
    ~TrajectoryController() = default;

    virtual float calculateVelocity(const VehicleState &state) = 0;
    virtual float calculateDelta(const VehicleState &state) = 0;
};

class InterpolatingTrajectoryController : public TrajectoryController
{
public:
    InterpolatingTrajectoryController(ReferencePath<1000> path, VehicleSpec spec) : m_path(std::move(path)), m_spec(spec) {}
    ~InterpolatingTrajectoryController() = default;

    float calculateVelocity(const VehicleState &state) override
    {
        static constexpr auto vel_max = 0.5f;
        static constexpr auto vel_min = 0.1f;
        static constexpr auto interval = 0.3f;
        static const auto s_max = m_path.ref_s.back();
        static const auto s_interval = s_max * interval;

        float vel = 0.0f;
        if (state.s < s_interval)
        {
            vel = -std::max(vel_min, vel_max * state.s / s_interval);
        }
        else if (state.s < s_max - s_interval)
        {
            vel = -vel_max;
        }
        else
        {
            vel = -std::max(vel_max * (s_max - state.s) / s_interval, 0.0f);
        }

        return vel;
    }

    float calculateDelta(const VehicleState &state) override
    {
        const auto cur = m_path.interpolate_ref_s(m_path.ref_cur, state.s);
        return -std::atan2(m_spec.wb * cur, 1.0f);
    }

private:
    const ReferencePath<1000> m_path;
    const VehicleSpec m_spec;
};

class SlidingModeTrajectoryController : public TrajectoryController
{
public:
    SlidingModeTrajectoryController(ReferencePath<1000> path, VehicleSpec spec) : m_path(std::move(path)), m_spec(spec) {}
    ~SlidingModeTrajectoryController() = default;

    float calculateVelocity(const VehicleState &state) override
    {
        static constexpr auto vel_max = 0.5f;
        static constexpr auto vel_min = 0.1f;
        static constexpr auto interval = 0.3f;
        static const auto s_max = m_path.ref_s.back();
        static const auto s_interval = s_max * interval;

        float vel = 0.0f;
        if (state.s < s_interval)
        {
            vel = -std::max(vel_min, vel_max * state.s / s_interval);
        }
        else if (state.s < s_max - s_interval)
        {
            vel = -vel_max;
        }
        else
        {
            vel = -std::max(vel_max * (s_max - state.s) / s_interval, 0.0f);
        }

        return vel;
    }

    float calculateDelta(const VehicleState &state) override
    {
        // negative sign is required since we are moving in the direction of -x
        const auto ref_cur = -m_path.interpolate_ref_x(m_path.ref_cur, state.x);
        const auto ref_phi = m_path.interpolate_ref_x(m_path.ref_phi, state.x);
        const auto ref_y = m_path.interpolate_ref_x(m_path.ref_y, state.x);

        // kappa_i > 0 are a design parameter
        float kappa_0 = 1.0f;
        float kappa_1 = 1.0f;
        float kappa_2 = 0.01f;

        float y_err = ref_y - state.y;
        float sigma = kappa_0 * y_err + (std::tan(state.phi) - std::tan(ref_phi));

        auto sign = [](float x, float band = 0.01f) -> float
        { return x > band ? 1.0f : x > -band ? x / band
                                             : -1.0f; };
        float delta = std::atan2(m_spec.wb * std::pow(std::cos(state.phi), 3) * (ref_cur / std::pow(std::cos(ref_phi), 3) + kappa_0 * (std::tan(state.phi) - std::tan(ref_phi)) + kappa_1 * sigma + kappa_2 * sign(sigma)), 1.0f);

        return delta;
    }

private:
    const ReferencePath<1000> m_path;
    const VehicleSpec m_spec;
};

class PITrajectoryController : public TrajectoryController
{
public:
    PITrajectoryController(const VehicleState &vehicleState, VehicleSpec spec) : x_max(vehicleState.x), m_spec(spec) {}
    ~PITrajectoryController() = default;

    float calculateVelocity(const VehicleState &state) override
    {
        static constexpr auto vel_max = 0.5f;
        static constexpr auto vel_min = 0.1f;
        static constexpr auto interval = 0.3f;
        static const auto x_interval = x_max * interval;

        float vel = 0.0f;
        if (state.x > x_max - x_interval)
        {
            vel = -std::max(vel_min, vel_max * (x_max - state.x) / x_interval);
        }
        else if (state.x > x_interval)
        {
            vel = -vel_max;
        }
        else
        {
            vel = -std::max(vel_max * state.x / x_interval, 0.0f);
        }

        return vel;
    }

    float calculateDelta(const VehicleState &state) override
    {
        float theta_ref = std::atan2(state.y, state.x);
        std::cout << "theta_ref: " << theta_ref << std::endl;
        std::cout << "phi: " << state.phi << std::endl;
        float error = state.phi - theta_ref;
        std::cout << "error: " << error << std::endl;
        float delta = m_deltaController.calculate(error);
        std::cout << "delta: " << delta << std::endl;
        return delta;
    }

private:
    const float x_max;
    const VehicleSpec m_spec;
    PIController m_deltaController{2.0f, 0.0f, -0.5f, 0.5f};
};

#endif // CONTROLLER_INC_TRAJECTORYCONTROLLER_H