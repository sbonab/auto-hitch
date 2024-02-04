#include "InputController.hpp"

int main()
{
    const std::string inputPipePath = "/tmp/vehicle_input.pipe";
    const std::string outputPipePath = "/tmp/vehicle_output.pipe";
    const InputPlanner inputPlanner{};
    InputController inputController(inputPipePath, outputPipePath, inputPlanner);

    inputController.start();

    return 0;
}