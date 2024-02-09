#ifndef CONTROLLER_INC_POINT_HPP
#define CONTROLLER_INC_POINT_HPP

struct Point
{
    float x;
    float y;
};

float distance(const Point &p1, const Point &p2);

Point operator+(const Point &p1, const Point &p2);

Point operator-(const Point &p1, const Point &p2);

Point operator*(const Point &p, float scalar);

#endif // CONTROLLER_INC_POINT_HPP