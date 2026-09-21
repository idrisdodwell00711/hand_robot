#ifndef HAND_TELEOP_GLOVE_KINEMATICS_HPP
#define HAND_TELEOP_GLOVE_KINEMATICS_HPP

#include <vector>


class GloveKinematics
{
public:

    GloveKinematics();


    std::vector<double> calculateJointAngles(
        const std::vector<double> &sensor_values
    );
};


#endif