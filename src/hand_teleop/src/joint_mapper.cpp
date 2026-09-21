#include "hand_teleop/joint_mapper.hpp"


JointMapper::JointMapper()
{
}


std::vector<double>
JointMapper::mapToRobot(
    const std::vector<double> &human_joints)
{
    std::vector<double> robot_joints;


    // ---------------------------------------------------------
    // The robot currently has 14 joints.
    //
    // Ordering currently follows joint_limits.yaml:
    //
    // 0  base_to_base_1
    // 1  base_to_base_2
    // 2  base_to_base_3
    // 3  base_to_base_4
    // 4  base_to_base_thumb
    // 5  base_to_middle_1
    // 6  base_to_middle_2
    // 7  base_to_middle_3
    // 8  base_to_middle_4
    // 9  middle_to_tip_1
    // 10 middle_to_tip_2
    // 11 middle_to_tip_3
    // 12 middle_to_tip_4
    // 13 thumb_base_to_middle
    //
    // This ordering should be verified against the URDF/SRDF
    // before connecting the physical glove.
    // ---------------------------------------------------------


    if (human_joints.size() != 14)
    {
        return robot_joints;
    }


    robot_joints.resize(14);


    for (std::size_t i = 0; i < 14; ++i)
    {
        robot_joints[i] =
            human_joints[i];
    }


    return robot_joints;
}