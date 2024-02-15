#ifndef CONTROLLER_INC_REFERENECPATH_HPP
#define CONTROLLER_INC_REFERENECPATH_HPP

#include <array>
#include <algorithm>
#include "Vehicle.hpp"
#include <iostream>

template <std::size_t N>
struct ReferencePath
{
    // ref_s must be strictly ascending and ref_x be strictly descending
    // displacement along the path
    std::array<float, N> ref_s;
    // x-coordinate of the rear axle
    std::array<float, N> ref_x;
    // y-coordinate of the rear axle
    std::array<float, N> ref_y;
    // vehicle angle with respect to the x-axis
    std::array<float, N> ref_phi;
    // curvature of the path
    std::array<float, N> ref_cur;

    float interpolate_ref_s(const std::array<float, N> &target, float s) const
    {
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

        std::size_t index = std::distance(ref_s.begin(), it);

        // Calculate the interpolation factor
        float factor = (s - ref_s[index - 1]) / (ref_s[index] - ref_s[index - 1]);

        // Interpolate the value in ref
        float interpolatedValue = target[index - 1] + factor * (target[index] - target[index - 1]);

        return interpolatedValue;
    }

    float interpolate_ref_x(const std::array<float, N> &target, float x) const
    {
        // ref_x is strictly descending
        auto it = std::lower_bound(ref_x.begin(), ref_x.end(), x, std::greater<float>());

        // Check if x is out of range
        if (it == ref_x.begin())
        {
            return target.front();
        }

        if (it == ref_x.end())
        {
            return target.back();
        }

        // Get the index of the lower bound value
        std::size_t index = std::distance(ref_x.begin(), it);

        // Calculate the interpolation factor
        float factor = (x - ref_x[index - 1]) / (ref_x[index] - ref_x[index - 1]);

        // Interpolate the value in ref
        float interpolatedValue = target[index - 1] + factor * (target[index] - target[index - 1]);

        return interpolatedValue;
    }
};

#endif // CONTROLLER_INC_REFERENECPATH_HPP