#ifndef CONTROLLER_INC_INPUTCONTROLLER_HPP
#define CONTROLLER_INC_INPUTCONTROLLER_HPP

#include <string>
#include <atomic>
#include <thread>
#include <optional>
#include "InputSchedule.hpp"
#include "Vehicle.hpp"

class InputController
{
public:
    InputController(const std::string &inputPipePath, const std::string &outputPipePath);
    ~InputController();

    void start();

    void stop();

private:
    std::string m_inputPipePath;
    std::string m_outputPipePath;
    std::atomic<bool> m_isRunning;
    std::thread m_writerThread;
    std::thread m_readerThread;
    std::optional<Vehicle> m_vehicle;
    std::optional<InputSchedule<1000>> m_inputSchedule;

    void calculateAndUpdateInput(std::size_t freq = 50);
    void readAndUpdateOutput();
    void calculateInputSchedule(const Vehicle &vehicle);
};

#endif // CONTROLLER_INC_INPUTCONTROLLER_HPP