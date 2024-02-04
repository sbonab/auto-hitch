#ifndef CONTROLLER_INC_INPUTPLANNER_HPP
#define CONTROLLER_INC_INPUTPLANNER_HPP
#include "InputSchedule.hpp"
#include "Vehicle.hpp"

class InputPlanner
{
public:
    InputPlanner() = default;
    ~InputPlanner() = default;

    template <std::size_t N>
    InputSchedule<N> createSchedule(const Vehicle &vehicle) const
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
};

#endif // CONTROLLER_INC_INPUTPLANNER_HPP
