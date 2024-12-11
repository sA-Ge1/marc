#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_point");

    // Create MoveGroup interface for arm control
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    // Set the desired position for the end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.5; // Replace with your desired x-coordinate
    target_pose.position.y = 0.2; // Replace with your desired y-coordinate
    target_pose.position.z = 0.3; // Replace with your desired z-coordinate

    // Setting the orientation as identity quaternion (no rotation)
    target_pose.orientation.w = 1.0;

    // Set the target pose to the MoveGroup interface
    move_group.setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = move_group.plan(my_plan);

    if (success)
    {
        // Execute the planned motion
        move_group.move();
        RCLCPP_INFO(node->get_logger(), "Moved to target position!");
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning failed.");
    }

    rclcpp::shutdown();
    return 0;
}
