#include "InputController.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <future>

InputController::InputController(const std::string &pipePath) : m_pipePath(pipePath), m_isRunning(false)
{
}

InputController::~InputController()
{
    stop();
}

void InputController::start()
{
    m_isRunning = true;
    auto done = std::async(&InputController::writeToPipe, this, 10);
    done.wait();
}

void InputController::stop()
{
    m_isRunning = false;
}

void InputController::writeToPipe(std::size_t freq)
{
    int pipeFd = open(m_pipePath.c_str(), O_WRONLY);
    if (pipeFd == -1)
    {
        std::cerr << "Failed to open pipe" << std::endl;
        return;
    }
    while (m_isRunning)
    {
        float velocity = 1.0f;
        float angle = 0.5f;

        std::string message = std::to_string(velocity) + "," + std::to_string(angle) + "\n";
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
