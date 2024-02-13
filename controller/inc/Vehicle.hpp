#ifndef CONTROLLER_INC_VEHICLE_HPP
#define CONTROLLER_INC_VEHICLE_HPP

struct VehicleState
{
    float x;
    float y;
    float theta;
    float s;
};

struct VehicleSpec
{
    // wheelbase
    float wb;
    // turning radius
    float radius;
};

#endif // CONTROLLER_INC_VEHICLE_HPP