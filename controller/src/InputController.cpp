#include "InputController.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <future>

InputController::InputController(std::string inputPipePath, std::string outputPipePath, VehicleSpec vehicleSpec)
    : m_inputPipePath(std::move(inputPipePath)), m_outputPipePath(std::move(outputPipePath)), m_vehicleSpec(std::move(vehicleSpec)), m_pathPlanner(PathPlanner()), m_isRunning(false)
{
}

InputController::~InputController()
{
    stop();
}

void InputController::start()
{
    m_isRunning = true;
    auto doneUpdatingInput = std::async(&InputController::calculateAndUpdateInput, this, 10);
    auto doneUpdatingOutput = std::async(&InputController::readAndUpdateOutput, this);
    doneUpdatingInput.wait();
}

void InputController::stop()
{
    m_isRunning = false;
}

void InputController::calculateAndUpdateInput(std::size_t freq)
{
    int pipeFd = open(m_inputPipePath.c_str(), O_WRONLY);
    if (pipeFd == -1)
    {
        std::cerr << "Failed to open pipe" << std::endl;
        return;
    }
    while (m_isRunning)
    {
        const auto sleepTime{std::chrono::milliseconds(1000 / freq)};
        decltype(m_vehicleState) vehicleState{};
        {
            std::lock_guard<std::mutex> lock(m_vehicleStateMutex);
            vehicleState = m_vehicleState;
        }

        if (!vehicleState)
        {
            std::this_thread::sleep_for(sleepTime);
            continue;
        }

        float velocity = m_trajectoryController->calculateVelocity(*vehicleState);
        float alpha = m_trajectoryController->calculateAlpha(*vehicleState);

        std::string message = std::to_string(velocity) + " " + std::to_string(alpha) + "\n";
        write(pipeFd, message.c_str(), message.size());
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << std::put_time(std::localtime(&now_c), "%c") << " - Sent: " << message;
        fsync(pipeFd);
        std::this_thread::sleep_for(sleepTime);
    }
    close(pipeFd);
}

void InputController::readAndUpdateOutput()
{
    int pipeFd = open(m_outputPipePath.c_str(), O_RDONLY);
    if (pipeFd == -1)
    {
        std::cerr << "Failed to open pipe" << std::endl;
        return;
    }
    while (m_isRunning)
    {
        char buffer[1024];
        auto bytesRead = read(pipeFd, buffer, sizeof(buffer));
        if (bytesRead > 0)
        {
            buffer[bytesRead] = '\0';
            float x, y, phi, s;
            std::istringstream iss(buffer);
            iss >> x >> y >> phi >> s;
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << std::put_time(std::localtime(&now_c), "%c") << " - Received: "
                      << "x: " << x << ", y: " << y << ", theta: " << phi << ", s: " << s << "\n";
            VehicleState vehicleState{x, y, phi, s};

            std::lock_guard<std::mutex> lock(m_vehicleStateMutex);
            if (!m_vehicleState)
            {
                std::cout << "Calculating input schedule for the first time" << std::endl;
                initTrajectoryController(vehicleState);
            }
            m_vehicleState = vehicleState; // Update vehicle state
        }
    }
    close(pipeFd);
}

void InputController::initTrajectoryController(const VehicleState &vehicleState)
{
    // m_trajectoryController = std::make_unique<InterpolatingTrajectoryController>(m_pathPlanner.createPath<1000>(vehicleState, m_vehicleSpec), m_vehicleSpec);
    m_trajectoryController = std::make_unique<SlidingModeTrajectoryController>(m_pathPlanner.createPath<1000>(vehicleState, m_vehicleSpec), m_vehicleSpec);
}