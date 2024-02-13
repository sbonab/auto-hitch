#ifndef CONTROLLER_INC_REFERENECPATH_HPP
#define CONTROLLER_INC_REFERENECPATH_HPP

#include <array>
#include <algorithm>
#include "Vehicle.hpp"

template <std::size_t N>
struct ReferencePath
{
    // ref_s must be strictly ascending and ref_x be strictly descending
    std::array<float, N> ref_s;
    std::array<float, N> ref_x;
    std::array<float, N> ref_y;
    std::array<float, N> ref_cur;

    float interpolate_ref_s(const std::array<float, N> &target, float s) const
    {
        // Find the index of the lower bound value in sch_s
        auto it = std::upper_bound(ref_s.begin(), ref_s.end(), s);

        // Check if s is out of range
        if (it == ref_s.begin())
        {
            return target.front();
        }

        if (it == ref_s.end())
        {
            return target.back();
        }

        // Get the index of the lower bound value
        std::size_t index = std::distance(ref_s.begin(), it);

        // Calculate the interpolation factor
        float factor = (s - ref_s[index - 1]) / (ref_s[index] - ref_s[index - 1]);

        // Interpolate the value in ref
        float interpolatedValue = target[index - 1] + factor * (target[index] - target[index - 1]);

        return interpolatedValue;
    }
};

#endif // CONTROLLER_INC_REFERENECPATH_HPP