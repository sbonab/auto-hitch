#ifndef CONTROLLER_INC_INPUTSCHEDULE_HPP
#define CONTROLLER_INC_INPUTSCHEDULE_HPP

#include <array>
#include <algorithm>
#include "Vehicle.hpp"

template <std::size_t N>
struct InputSchedule
{
    // sch_s must be strictly ascending, otherwise the interpolation will not work
    std::array<float, N> sch_s;
    std::array<float, N> sch_vel;
    std::array<float, N> sch_alpha;

    float calculateVelocity(float s) const
    {
        return interpolate(sch_vel, s);
    }

    float calculateAlpha(float s) const
    {
        return interpolate(sch_alpha, s);
    }

private:
    float interpolate(const std::array<float, N> &sch, float s) const
    {
        // Find the index of the lower bound value in sch_s
        auto it = std::upper_bound(sch_s.begin(), sch_s.end(), s);

        // Check if s is out of range
        if (it == sch_s.begin())
        {
            return sch.front();
        }

        if (it == sch_s.end())
        {
            return sch.back();
        }

        // Get the index of the lower bound value
        std::size_t index = std::distance(sch_s.begin(), it);

        // Calculate the interpolation factor
        float factor = (s - sch_s[index - 1]) / (sch_s[index] - sch_s[index - 1]);

        // Interpolate the value in sch
        float interpolatedValue = sch[index - 1] + factor * (sch[index] - sch[index - 1]);

        return interpolatedValue;
    }
};

#endif // CONTROLLER_INC_INPUTSCHEDULE_HPP