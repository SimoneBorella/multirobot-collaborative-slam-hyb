#ifndef TYPES_H
#define TYPES_H

#include <string>

struct Point
{
    double x, y;
};

struct Position
{
    double x, y, theta;
};

struct Shape
{
    std::string type;
    double diameter;
};

struct Command
{
    double linear_vel, angular_vel;
};


#endif // TYPES_H
