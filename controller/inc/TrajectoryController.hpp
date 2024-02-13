#ifndef CONTROLLER_INC_TRAJECTORYCONTROLLER_H
#define CONTROLLER_INC_TRAJECTORYCONTROLLER_H

#include "ReferencePath.hpp"
#include "Vehicle.hpp"
#include <algorithm>
#include <cmath>

class TrajectoryController
{
public:
    TrajectoryController() = default;
    ~TrajectoryController() = default;

    virtual float calculateVelocity(const VehicleState &state) const = 0;
    virtual float calculateAlpha(const VehicleState &state) const = 0;
};

class InterpolatingTrajectoryController : public TrajectoryController
{
public:
    InterpolatingTrajectoryController(ReferencePath<1000> path, VehicleSpec spec) : m_path(std::move(path)), m_spec(spec) {}
    ~InterpolatingTrajectoryController() = default;

    float calculateVelocity(const VehicleState &state) const override
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

    float calculateAlpha(const VehicleState &state) const override
    {
        const auto cur = m_path.interpolate_ref_s(m_path.ref_cur, state.s);
        return -std::atan2(m_spec.wb * cur, 1.0f);
    }

private:
    const ReferencePath<1000> m_path;
    const VehicleSpec m_spec;
};

#endif // CONTROLLER_INC_TRAJECTORYCONTROLLER_H