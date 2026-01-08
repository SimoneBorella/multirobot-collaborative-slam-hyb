#ifndef TYPES_H
#define TYPES_H

#include <string>

struct Point
{
    double x, y;
    Point(double x = 0.0, double y = 0.0)
        : x(x), y(y) {}
};

struct Position
{
    double x, y, theta;
    Position(double x = 0.0, double y = 0.0, double theta = 0.0)
        : x(x), y(y), theta(theta) {}
};

struct Shape
{
    std::string type;
    double diameter;
    Shape(const std::string& type = "", double diameter = 0.0)
        : type(type), diameter(diameter) {}
};

struct Command
{
    double linear_vel, angular_vel;
    Command(double lin = 0.0, double ang = 0.0)
        : linear_vel(lin), angular_vel(ang) {}
};


#endif // TYPES_H
