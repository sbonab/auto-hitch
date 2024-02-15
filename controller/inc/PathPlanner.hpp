#ifndef CONTROLLER_INC_PATHPLANNER_HPP
#define CONTROLLER_INC_PATHPLANNER_HPP

#include "ReferencePath.hpp"
#include "Vehicle.hpp"
#include <cmath>
#include "Point.hpp"
#include <iostream>

class PathPlanner
{
public:
    PathPlanner() = default;
    ~PathPlanner() = default;

    /**
     * @brief Creates a circular arc based trajectory for the vehicle to go from the current position to (0, 0)
     *
     * This method generates a schedule of positions, velocities, and alpha values along the x-axis for a given vehicle.
     * The schedule is created based on the maximum x-coordinate of the vehicle and the desired number of points (N).
     *
     * @tparam N The number of points in the schedule.
     * @param vehicle The vehicle for which the schedule is created.
     * @param radius Turning radius to be used for planning path
     * @param wb Wheelbase of the vehicle
     * @param vel_max The maximum longitudinal vehicle velocity to be used fo planning
     * @return An ReferencePath<N> object representing the generated schedule.
     */

    template <std::size_t N>
    ReferencePath<N> createPath(const VehicleState &state, const VehicleSpec &spec) const
    {
        std::array<float, N> ref_s;
        std::array<float, N> ref_x;
        std::array<float, N> ref_y;
        std::array<float, N> ref_phi;
        std::array<float, N> ref_cur;

        const auto &radius = spec.radius;
        const auto &wb = spec.wb;

        auto p_target = Point{0.0f, 0.0f};
        auto p_veh = Point{state.x, state.y};
        Point p_O_target = p_target + Point{0, p_veh.y > p_target.y ? radius : -radius};
        Point p_O_veh = p_veh + Point{0, p_veh.y > p_target.y ? -radius : radius};
        // Distance between two circle centers
        float d_OO = distance(p_O_target, p_O_veh);
        // Distance between two tangent points
        float d_GG = 2 * std::sqrt(std::pow(d_OO / 2, 2) - std::pow(radius, 2));
        // Angle of tangent line
        float theta = std::abs(std::asin((p_veh.x - p_target.x) / d_OO) - std::acos(2 * radius / d_OO));

        float s_theta = radius * theta;
        float s_max = 2 * s_theta + d_GG;

        // Utility coefficient to determine the direction of the arc
        auto coeff = p_veh.y > p_target.y ? 1 : -1;
        for (std::size_t i = 0; i < N; ++i)
        {
            float s = s_max * i / N;
            ref_s[i] = s;

            if (s < s_theta)
            {
                float ang_arc = s / radius;
                ref_x[i] = p_veh.x - radius * std::sin(ang_arc);
                ref_y[i] = p_veh.y - coeff * (radius - radius * std::cos(ang_arc));
                ref_phi[i] = coeff * ang_arc;
                ref_cur[i] = coeff * 1.0f / radius;
            }
            else if (s < s_max - s_theta)
            {
                ref_x[i] = p_veh.x - radius * std::sin(theta) - (s - s_theta) * std::cos(theta);
                ref_y[i] = p_veh.y - coeff * (radius - radius * std::cos(theta)) - coeff * (s - s_theta) * std::sin(theta);
                ref_phi[i] = coeff * theta;
                ref_cur[i] = 0.0f;
            }
            else
            {
                float ang_arc = (s_max - s) / radius;
                ref_x[i] = p_target.x + radius * std::sin(ang_arc);
                ref_y[i] = p_target.y + coeff * (radius - radius * std::cos(ang_arc));
                ref_phi[i] = coeff * ang_arc;
                ref_cur[i] = -coeff * 1.0f / radius;
            }
        }

        return ReferencePath<N>{ref_s, ref_x, ref_y, ref_phi, ref_cur};
    }
};

#endif // CONTROLLER_INC_PATHPLANNER_HPP
