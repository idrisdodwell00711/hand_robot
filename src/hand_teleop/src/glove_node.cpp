#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hand_interfaces/msg/joint_command.hpp>

#include "hand_teleop/glove_driver.hpp"
#include "hand_teleop/glove_kinematics.hpp"
#include "hand_teleop/joint_mapper.hpp"


using JointCommand =
    hand_interfaces::msg::JointCommand;


class GloveNode : public rclcpp::Node
{
public:

    GloveNode()
        : Node("glove_node")
    {
        // ---------------------------------------------------------
        // Publisher
        // ---------------------------------------------------------

        joint_command_pub_ =
            this->create_publisher<JointCommand>(
                "joint_command",
                10
            );


        // ---------------------------------------------------------
        // Glove components
        // ---------------------------------------------------------

        glove_driver_ =
            std::make_shared<GloveDriver>();


        glove_kinematics_ =
            std::make_shared<GloveKinematics>();


        joint_mapper_ =
            std::make_shared<JointMapper>();


        // ---------------------------------------------------------
        // Initialise glove
        // ---------------------------------------------------------

        if (!glove_driver_->initialize())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to initialise glove"
            );

            return;
        }


        // ---------------------------------------------------------
        // Update timer
        //
        // 20 ms = 50 Hz
        // ---------------------------------------------------------

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &GloveNode::update,
                    this
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "Glove teleoperation node started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing 14-joint commands at 50 Hz"
        );
    }


    ~GloveNode()
    {
        if (glove_driver_)
        {
            glove_driver_->shutdown();
        }
    }


private:

    // =============================================================
    // Update
    // =============================================================

    void update()
    {
        // ---------------------------------------------------------
        // Read glove sensors
        // ---------------------------------------------------------

        std::vector<double> sensor_values;


        if (!glove_driver_->readSensors(
                sensor_values))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to read glove sensors"
            );

            return;
        }


        // ---------------------------------------------------------
        // Calculate human joint angles
        // ---------------------------------------------------------

        std::vector<double> human_joints =
            glove_kinematics_->calculateJointAngles(
                sensor_values
            );


        // ---------------------------------------------------------
        // Map human joints to robot joints
        // ---------------------------------------------------------

        std::vector<double> robot_joints =
            joint_mapper_->mapToRobot(
                human_joints
            );


        // ---------------------------------------------------------
        // Check robot joint count
        // ---------------------------------------------------------

        if (robot_joints.size() != 14)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Expected 14 robot joints, received %zu",
                robot_joints.size()
            );

            return;
        }


        // ---------------------------------------------------------
        // Create JointCommand
        // ---------------------------------------------------------

        JointCommand msg;


        msg.positions =
            robot_joints;


        // ---------------------------------------------------------
        // Publish
        // ---------------------------------------------------------

        joint_command_pub_->publish(msg);
    }


    // =============================================================
    // Publisher
    // =============================================================

    rclcpp::Publisher<JointCommand>::SharedPtr
        joint_command_pub_;


    // =============================================================
    // Glove components
    // =============================================================

    std::shared_ptr<GloveDriver>
        glove_driver_;


    std::shared_ptr<GloveKinematics>
        glove_kinematics_;


    std::shared_ptr<JointMapper>
        joint_mapper_;


    // =============================================================
    // Timer
    // =============================================================

    rclcpp::TimerBase::SharedPtr
        timer_;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<GloveNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}