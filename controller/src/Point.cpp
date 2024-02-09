#include "Point.hpp"
#include <cmath>
float distance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

Point operator+(const Point &p1, const Point &p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

Point operator-(const Point &p1, const Point &p2)
{
    return {p1.x - p2.x, p1.y - p2.y};
}

Point operator*(const Point &p, float scalar)
{
    return {p.x * scalar, p.y * scalar};
}
