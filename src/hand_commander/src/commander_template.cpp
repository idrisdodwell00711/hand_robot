#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <example_interfaces/msg/bool.hpp>

#include <hand_interfaces/msg/joint_command.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.hpp>


using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using std::placeholders::_1;

using Bool =
    example_interfaces::msg::Bool;

using JointCommand =
    hand_interfaces::msg::JointCommand;


class Commander
{
public:

    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;


        // ---------------------------------------------------------
        // MoveIt planning group
        // ---------------------------------------------------------

        hand_ =
            std::make_shared<MoveGroupInterface>(
                node_,
                "hand"
            );


        // ---------------------------------------------------------
        // Motion scaling
        // ---------------------------------------------------------

        hand_->setMaxVelocityScalingFactor(1.0);

        hand_->setMaxAccelerationScalingFactor(1.0);


        // ---------------------------------------------------------
        // Close hand subscriber
        // ---------------------------------------------------------

        close_hand_sub_ =
            node_->create_subscription<Bool>(
                "close_hand",
                10,
                std::bind(
                    &Commander::closeHandCallback,
                    this,
                    _1
                )
            );


        // ---------------------------------------------------------
        // Joint command subscriber
        // ---------------------------------------------------------

        joint_cmd_sub_ =
            node_->create_subscription<JointCommand>(
                "joint_command",
                10,
                std::bind(
                    &Commander::jointCommandCallback,
                    this,
                    _1
                )
            );


        RCLCPP_INFO(
            node_->get_logger(),
            "Hand commander started"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "Waiting for 14-joint commands..."
        );
    }


    // =============================================================
    // Move to named target
    // =============================================================

    void goToNamedTarget(
        const std::string &name)
    {
        hand_->setStartStateToCurrentState();

        hand_->setNamedTarget(name);

        planAndExecute(hand_);
    }


    // =============================================================
    // Move to joint target
    // =============================================================

    void goToJointTarget(
        const std::vector<double> &joints)
    {
        hand_->setStartStateToCurrentState();

        hand_->setJointValueTarget(joints);

        planAndExecute(hand_);
    }


    // =============================================================
    // Move to pose target
    // =============================================================

    void goToPoseTarget(
        double x,
        double y,
        double z,
        double pitch,
        double roll,
        double yaw)
    {
        tf2::Quaternion q;

        q.setRPY(
            roll,
            pitch,
            yaw
        );

        q.normalize();


        geometry_msgs::msg::PoseStamped target_pose;

        target_pose.header.frame_id =
            "base_link";


        target_pose.pose.position.x =
            x;

        target_pose.pose.position.y =
            y;

        target_pose.pose.position.z =
            z;


        target_pose.pose.orientation.x =
            q.getX();

        target_pose.pose.orientation.y =
            q.getY();

        target_pose.pose.orientation.z =
            q.getZ();

        target_pose.pose.orientation.w =
            q.getW();


        hand_->setStartStateToCurrentState();

        hand_->setPoseTarget(target_pose);

        planAndExecute(hand_);
    }


private:

    // =============================================================
    // Plan and execute
    // =============================================================

    void planAndExecute(
        std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;


        bool success =
            (
                interface->plan(plan)
                ==
                moveit::core::MoveItErrorCode::SUCCESS
            );


        if (success)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Plan successful. Executing..."
            );

            interface->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Planning failed!"
            );
        }
    }


    // =============================================================
    // Close hand callback
    // =============================================================

    void closeHandCallback(
        const Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Close hand command received"
            );

            // Put your hand-closing movement here.
        }
    }


    // =============================================================
    // Joint command callback
    // =============================================================

    void jointCommandCallback(
        const JointCommand::SharedPtr msg)
    {
        const auto &joints =
            msg->positions;


        // ---------------------------------------------------------
        // Check that exactly 14 joint values were received
        // ---------------------------------------------------------

        if (joints.size() == 14)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Received 14 joint commands"
            );

            goToJointTarget(joints);
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Expected 14 joint values, received %zu",
                joints.size()
            );
        }
    }


    // =============================================================
    // Member variables
    // =============================================================

    std::shared_ptr<rclcpp::Node>
        node_;


    std::shared_ptr<MoveGroupInterface>
        hand_;


    rclcpp::Subscription<Bool>::SharedPtr
        close_hand_sub_;


    rclcpp::Subscription<JointCommand>::SharedPtr
        joint_cmd_sub_;
};


// =================================================================
// Main
// =================================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<rclcpp::Node>(
            "commander"
        );


    auto commander =
        std::make_shared<Commander>(
            node
        );


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}