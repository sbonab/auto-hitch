#ifndef CONTROLLER_INC_INPUTCONTROLLER_HPP
#define CONTROLLER_INC_INPUTCONTROLLER_HPP

#include <string>
#include <atomic>
#include <thread>

class InputController
{
public:
    InputController(const std::string &pipePath);
    ~InputController();

    void start();

    void stop();

private:
    std::string m_pipePath;
    std::atomic<bool> m_isRunning;
    std::thread m_writerThread;

    void writeToPipe(std::size_t freq = 10);
};

#endif // CONTROLLER_INC_INPUTCONTROLLER_HPP