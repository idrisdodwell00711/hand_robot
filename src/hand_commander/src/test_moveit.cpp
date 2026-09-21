#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <thread>
#include <vector>

int main(int argc, char** argv)
{
    // Start ROS
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    // Start ROS executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    // Connect to MoveIt
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");

    // Movement speed
    hand.setMaxVelocityScalingFactor(1.0);
    hand.setMaxAccelerationScalingFactor(1.0);

    // ------------------------------------------------------------
    // JOINT TARGET
    //
    // 0  base_to_base_thumb
    // 1  thumb_base_to_middle
    //
    // 2  base_to_base_1
    // 3  base_to_middle_1
    // 4  middle_to_tip_1
    //
    // 5  base_to_base_2
    // 6  base_to_middle_2
    // 7  middle_to_tip_2
    //
    // 8  base_to_base_3
    // 9  base_to_middle_3
    // 10 middle_to_tip_3
    //
    // 11 base_to_base_4
    // 12 base_to_middle_4
    // 13 middle_to_tip_4
    // ------------------------------------------------------------

    std::vector<double> joints =
    {
        0.20, 0.20,       // Thumb

        0.20, 0.20, 0.20, // Finger 1
        0.20, 0.20, 0.20, // Finger 2
        0.20, 0.20, 0.20, // Finger 3
        0.20, 0.20, 0.20  // Finger 4
    };

    // Use the current robot position as the starting position
    hand.setStartStateToCurrentState();

    // Set the target
    if (!hand.setJointValueTarget(joints))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to set joint target.");

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Joint target set. Planning...");

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto result = hand.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Planning failed. Error code: %d",
            result.val);

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Planning successful. Executing...");

    // Execute
    auto execute_result = hand.execute(plan);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Execution successful.");
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Execution failed. Error code: %d",
            execute_result.val);
    }

    // Give ROS time to finish
    std::this_thread::sleep_for(
        std::chrono::seconds(1));

    // Stop ROS
    rclcpp::shutdown();
    spinner.join();

    return 0;
}