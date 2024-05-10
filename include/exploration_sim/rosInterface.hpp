// ros_visualizer.hpp
#ifndef ROS_VISUALIZER_HPP_
#define ROS_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "exploration_sim/helpers.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ServerGoalHandle<NavigateToPose>;
class RosInterface : public rclcpp::Node
{
public:
    RosInterface(std::shared_ptr<std::map<std::string, RobotCharacteristics>> robotMaps);

    void publishCostmap(cv::Mat image);

    void sendROSTransform();

    std::mutex* getRobotMutex()
    {
        return &robotMutex_;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav2> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNav2> goal_handle);

    void execute(const std::shared_ptr<GoalHandleNav2> goal_handle);

private:
    void threadFunction(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ptr)
    {
        ptr->spin();
    }

    rclcpp::Node::SharedPtr visualizer_node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> visualizer_executor_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr tfTimer_;
    std::mutex robotMutex_;
    std::mutex costmapMutex_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    std::vector<rclcpp_action::Server<NavigateToPose>::SharedPtr> nav2_action_server_vector_;
    std::shared_ptr<std::map<std::string, RobotCharacteristics>> robotMaps_;
};

#endif // ROS_VISUALIZER_HPP