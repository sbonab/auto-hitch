#ifndef CONTROLLER_INC_INPUTCONTROLLER_HPP
#define CONTROLLER_INC_INPUTCONTROLLER_HPP

#include <string>
#include <atomic>
#include <thread>
#include <optional>
#include "ReferencePath.hpp"
#include "Vehicle.hpp"
#include "PathPlanner.hpp"
#include "TrajectoryController.hpp"

class InputController
{
public:
    InputController(std::string inputPipePath, std::string outputPipePath, VehicleSpec vehicleSpec);
    ~InputController();

    void start();

    void stop();

private:
    std::string m_inputPipePath;
    std::string m_outputPipePath;
    VehicleSpec m_vehicleSpec;

    PathPlanner m_pathPlanner;
    std::unique_ptr<TrajectoryController> m_trajectoryController;

    std::atomic<bool> m_isRunning;
    std::thread m_writerThread;
    std::thread m_readerThread;
    std::optional<VehicleState> m_vehicleState;

    void calculateAndUpdateInput(std::size_t freq = 50);
    void readAndUpdateOutput();
    void calculateInputSchedule(const VehicleState &vehicle);
};

#endif // CONTROLLER_INC_INPUTCONTROLLER_HPP