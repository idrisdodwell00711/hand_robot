#ifndef HAND_TELEOP_JOINT_MAPPER_HPP
#define HAND_TELEOP_JOINT_MAPPER_HPP

#include <vector>


class JointMapper
{
public:

    JointMapper();


    std::vector<double> mapToRobot(
        const std::vector<double> &human_joints
    );
};


#endif