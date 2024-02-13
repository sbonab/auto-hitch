#include "InputController.hpp"
#include "Vehicle.hpp"

int main()
{
    const std::string inputPipePath = "/tmp/vehicle_input.pipe";
    const std::string outputPipePath = "/tmp/vehicle_output.pipe";
    const VehicleSpec vehicleSpec{3.6f, 6.0f};
    InputController inputController(inputPipePath, outputPipePath, vehicleSpec);

    inputController.start();

    return 0;
}