#include "InputController.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <future>

InputController::InputController(const std::string &inputPipePath, const std::string &outputPipePath, const InputPlanner &inputPlanner)
    : m_inputPipePath(inputPipePath), m_outputPipePath(outputPipePath), m_inputPlanner(inputPlanner), m_isRunning(false)
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
        if (!m_vehicle)
        {
            auto sleepTime = std::chrono::milliseconds(1000 / freq);
            std::this_thread::sleep_for(sleepTime);
        }

        float velocity = m_inputSchedule->calculateVelocity(m_vehicle->s);
        float alpha = m_inputSchedule->calculateAlpha(m_vehicle->s);

        std::string message = std::to_string(velocity) + " " + std::to_string(alpha) + "\n";
        write(pipeFd, message.c_str(), message.size());
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << std::put_time(std::localtime(&now_c), "%c") << " - Sent: " << message;
        fsync(pipeFd);
        auto sleepTime = std::chrono::milliseconds(1000 / freq);
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
            float x, y, theta, s;
            std::istringstream iss(buffer);
            iss >> x >> y >> theta >> s;
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << std::put_time(std::localtime(&now_c), "%c") << " - Received: "
                      << "x: " << x << ", y: " << y << ", theta: " << theta << ", s: " << s << "\n";
            Vehicle vehicle{x, y, theta, s};
            if (!m_vehicle)
            {
                std::cout << "Calculating input schedule for the first time" << std::endl;
                calculateInputSchedule(vehicle);
            }
            m_vehicle = vehicle;
        }
    }
    close(pipeFd);
}

void InputController::calculateInputSchedule(const Vehicle &vehicle)
{
    m_inputSchedule = m_inputPlanner.createSchedule<1000>(vehicle);
}