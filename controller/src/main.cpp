#include "InputController.hpp"

int main()
{
    const std::string inputPipePath = "/tmp/vehicle_input.pipe";
    const std::string outputPipePath = "/tmp/vehicle_output.pipe";
    InputController inputController(inputPipePath, outputPipePath);

    inputController.start();

    return 0;
}