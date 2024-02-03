#include "InputController.hpp"

int main()
{
    const std::string pipePath = "/tmp/vehicle_pipe";
    InputController inputController(pipePath);

    inputController.start();

    return 0;
}