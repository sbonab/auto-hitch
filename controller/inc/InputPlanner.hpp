#ifndef CONTROLLER_INC_INPUTPLANNER_HPP
#define CONTROLLER_INC_INPUTPLANNER_HPP

#include "InputSchedule.hpp"
#include "Vehicle.hpp"
#include <cmath>
#include "Point.hpp"

class InputPlanner
{
public:
    InputPlanner() = default;
    ~InputPlanner() = default;

    /**
     * @brief Creates a schedule along the x-axis for a given vehicle.
     *
     * This method generates a schedule of positions, velocities, and alpha values along the x-axis for a given vehicle.
     * The schedule is created based on the maximum x-coordinate of the vehicle and the desired number of points (N).
     *
     * @tparam N The number of points in the schedule.
     * @param vehicle The vehicle for which the schedule is created.
     * @return An InputSchedule<N> object representing the generated schedule.
     */
    template <std::size_t N>
    InputSchedule<N> createScheduleAlongX(const Vehicle &vehicle) const
    {
        std::array<float, N> sch_s;
        std::array<float, N> sch_vel;
        std::array<float, N> sch_alpha;

        float s_max = vehicle.x;
        float vel_max = 1.0f;
        for (std::size_t i = 0; i < N; ++i)
        {
            float s = s_max * i / N;
            sch_s[i] = s;
            float vel = s < s_max / 3       ? std::max(s * vel_max / (s_max / 3), vel_max / 10)
                        : s < 2 * s_max / 3 ? vel_max
                                            : (s_max - s) * vel_max / (s_max / 3);
            sch_vel[i] = -vel;
            sch_alpha[i] = 0.0f;
        }

        return {sch_s, sch_vel, sch_alpha};
    }

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
     * @return An InputSchedule<N> object representing the generated schedule.
     */

    template <std::size_t N>
    InputSchedule<N> createScheduleCurved(const Vehicle &vehicle, float radius, float wb, float vel_max) const
    {
        std::array<float, N> sch_s;
        std::array<float, N> sch_vel;
        std::array<float, N> sch_alpha;

        auto p_target = Point{0.0f, 0.0f};
        auto p_veh = Point{vehicle.x, vehicle.y};
        Point p_O_target = p_target + Point{0, p_veh.y > p_target.y ? radius : -radius};
        Point p_O_veh = p_veh + Point{0, p_veh.y > p_target.y ? -radius : radius};
        // Distance between two circle centers
        float d_OO = distance(p_O_target, p_O_veh);
        // Distance between two tangent points
        float d_GG = 2 * std::sqrt(std::pow(d_OO / 2, 2) - std::pow(radius, 2));
        // Angle of tangent line
        float theta = std::asin((p_veh.x - p_target.x) / d_OO) - std::acos(2 * radius / d_OO);

        float s_theta = radius * std::abs(theta);
        float s_max = 2 * s_theta + d_GG;
        for (std::size_t i = 0; i < N; ++i)
        {
            float s = s_max * i / N;
            sch_s[i] = s;
            float vel_min_start = 0.2f;
            float vel = s < s_theta           ? std::max(vel_max * s / s_theta, vel_min_start)
                        : s < s_max - s_theta ? vel_max
                                              : vel_max * (s_max - s) / s_theta;
            sch_vel[i] = -vel;
            auto alpha_sign = p_veh.y > p_target.y ? -1 : 1;
            sch_alpha[i] = s < s_theta           ? alpha_sign * std::atan2(wb, radius)
                           : s < s_max - s_theta ? 0.0f
                                                 : -alpha_sign * std::atan2(wb, radius);
        }

        return {sch_s, sch_vel, sch_alpha};
    }
};

#endif // CONTROLLER_INC_INPUTPLANNER_HPP
