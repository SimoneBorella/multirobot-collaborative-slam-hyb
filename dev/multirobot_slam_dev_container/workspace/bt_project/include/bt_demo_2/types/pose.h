#ifndef POSE_H
#define POSE_H

#include <string>

struct Pose
{
    double x;
    double y;
    double theta;
};


namespace BT
{
    template <> inline Pose convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }
}

#endif //POSE_H
