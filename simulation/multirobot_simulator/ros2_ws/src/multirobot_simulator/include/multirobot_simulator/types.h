#ifndef TYPES_H
#define TYPES_H

#include <string>

struct Point
{
    double x, y, z;
    Point(double x = 0.0, double y = 0.0, double z = 0.0)
        : x(x), y(y), z(z) {}
};

struct KeyPoint
{
    Point point;
    std::array<uint8_t, 32> descriptor;

    KeyPoint(double x = 0.0, double y = 0.0, double z = 0.0)
        : point(x, y, z) {}
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


struct BlindSpot
{
    double x, y, r;
};

struct Obstacle
{
    double x;
    double y;
    double theta;

    virtual ~Obstacle() = default;
};

struct ObstacleCircle : public Obstacle
{
    double r;
};

struct ObstacleRectangle : public Obstacle
{
    double w;
    double h;
};



#endif // TYPES_H
