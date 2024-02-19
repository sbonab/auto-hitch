#ifndef CONTROLLER_INC_PICONTROLLER_H
#define CONTROLLER_INC_PICONTROLLER_H

#include <optional>

class PIController
{
public:
    PIController(float kp, float ki) : m_kp(kp), m_ki(ki), m_integral(0.0f) {}
    PIController(float kp, float ki, float u_min, float u_max) : m_kp(kp), m_ki(ki), u_min(u_min), u_max(u_max), m_integral(0.0f) {}
    ~PIController() = default;

    // Assuming that the call for calculate is done at a constant rate, dt will be considered in the integral gain
    float calculate(float error)
    {
        m_integral += error;
        float controlInput = m_kp * error + m_ki * m_integral;

        bool saturated = false;
        if (u_min && controlInput < *u_min)
        {
            controlInput = *u_min;
            saturated = true;
        }
        if (u_max && controlInput > *u_max)
        {
            controlInput = *u_max;
            saturated = true;
        }
        // Let's not accumulate integral error if we are saturated
        if (saturated)
        {
            m_integral -= error;
        }

        return controlInput;
    }

    void reset()
    {
        m_integral = 0.0f;
    }

private:
    const float m_kp;
    const float m_ki;
    const std::optional<float> u_min;
    const std::optional<float> u_max;
    float m_integral;
};

#endif // CONTROLLER_INC_PICONTROLLER_H