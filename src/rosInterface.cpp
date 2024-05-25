// ros_visualizer.cpp
#include "exploration_sim/rosInterface.hpp"

RosInterface::RosInterface(std::shared_ptr<std::map<std::string, RobotCharacteristics>> robotMaps) : Node("ros_interface")
{
    robotMaps_ = robotMaps;

    visualizer_node_ = rclcpp::Node::make_shared("ros_interface_internal_node");
    visualizer_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    visualizer_executor_->add_node(visualizer_node_);

    costmap_publisher_ = visualizer_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("simulation_costmap", rclcpp::QoS(10).reliable().transient_local());

    // Initialize tf buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(visualizer_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(visualizer_node_);

    for (const auto &pair : *robotMaps_)
    {
        auto nav2_action_server_ = rclcpp_action::create_server<NavigateToPose>(visualizer_node_, pair.first + "/navigate_to_pose",
                                                                                std::bind(&RosInterface::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                                                std::bind(&RosInterface::handle_cancel, this, std::placeholders::_1),
                                                                                std::bind(&RosInterface::handle_accepted, this, std::placeholders::_1));
        nav2_action_server_vector_.push_back(nav2_action_server_);
    }
    // Create a timer that calls sendROSTransform every 0.1 seconds
    tfTimer_ = visualizer_node_->create_wall_timer(std::chrono::milliseconds(5), std::bind(&RosInterface::sendROSTransform, this));

    // spin the new executor in a new thread.
    std::thread t1(&RosInterface::threadFunction, this, visualizer_executor_);
    t1.detach();
    RCLCPP_INFO(visualizer_node_->get_logger(), "Ros visualizer constructor complete.");
}

rclcpp_action::GoalResponse RosInterface::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with x pos %f", goal->pose.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "Received goal request with y pos %f", goal->pose.pose.position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosInterface::handle_cancel(
    const std::shared_ptr<GoalHandleNav2> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RosInterface::handle_accepted(const std::shared_ptr<GoalHandleNav2> goal_handle)
{
    // using namespace std::placeholders;
    // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RosInterface::execute, this, std::placeholders::_1), goal_handle}.detach();
    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
}

void RosInterface::execute(const std::shared_ptr<GoalHandleNav2> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Executing goal of robot: " << goal_handle->get_goal()->behavior_tree);
    auto result = std::make_shared<NavigateToPose::Result>();
    getRobotMutex()->lock();
    (*robotMaps_)[goal_handle->get_goal()->behavior_tree].setPos.x = goal_handle->get_goal()->pose.pose.position.x;
    (*robotMaps_)[goal_handle->get_goal()->behavior_tree].setPos.y = goal_handle->get_goal()->pose.pose.position.y;
    getRobotMutex()->unlock();
    tf2::Quaternion q(
        goal_handle->get_goal()->pose.pose.orientation.x,
        goal_handle->get_goal()->pose.pose.orientation.y,
        goal_handle->get_goal()->pose.pose.orientation.z,
        goal_handle->get_goal()->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    getRobotMutex()->lock();
    (*robotMaps_)[goal_handle->get_goal()->behavior_tree].setPos.yaw = yaw;
    bool robotMoved = (*robotMaps_)[goal_handle->get_goal()->behavior_tree].setPos != (*robotMaps_)[goal_handle->get_goal()->behavior_tree].currentPos;
    getRobotMutex()->unlock();
    while (robotMoved)
    {
        getRobotMutex()->lock();
        robotMoved = (*robotMaps_)[goal_handle->get_goal()->behavior_tree].setPos != (*robotMaps_)[goal_handle->get_goal()->behavior_tree].currentPos;
        getRobotMutex()->unlock();
        std::cout << "Waiting for the robot to move" << std::endl;
        rclcpp::sleep_for(std::chrono::milliseconds(30));
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void RosInterface::publishCostmap(cv::Mat image)
{
    std::lock_guard<std::mutex> lock(costmapMutex_);
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.frame_id = "map";
    costmap.header.stamp = visualizer_node_->now();
    imageToOccupancy(costmap, 0.05, image);
    costmap_publisher_->publish(costmap);
}

void RosInterface::sendROSTransform()
{
    // RCLCPP_INFO(visualizer_node_->get_logger(), "SENDING TF");
    getRobotMutex()->lock();
    for (const auto &pair : *robotMaps_)
    {
        // RCLCPP_INFO(visualizer_node_->get_logger(), "Robot Name: %s, x: %f, y: %f, yaw: %f",
        //             pair.first.c_str(), pair.second.currentPos.x, pair.second.currentPos.y, pair.second.currentPos.yaw);
        // Publish transformation between frames
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = visualizer_node_->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = pair.first + "/base_link"; // Adjust child frame id as needed
        // Set the transformation values
        transformStamped.transform.translation.x = pair.second.currentPos.x;
        transformStamped.transform.translation.y = pair.second.currentPos.y;

        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(0.0, 0.0, pair.second.currentPos.yaw);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        transformStamped.transform.rotation = quaternion;

        // Publish the transformation
        tf_broadcaster_->sendTransform(transformStamped);
    }
    getRobotMutex()->unlock();
}