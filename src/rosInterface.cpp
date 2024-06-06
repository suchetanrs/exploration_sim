// ros_visualizer.cpp
#include "exploration_sim/rosInterface.hpp"

RosInterface::RosInterface(std::shared_ptr<std::map<std::string, RobotCharacteristics>> robotMaps, std::string &movementMethod) : Node("ros_interface")
{
    robotMaps_ = robotMaps;
    movementMethod_ = movementMethod;

    visualizer_node_ = rclcpp::Node::make_shared("ros_interface_internal_node");
    visualizer_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    visualizer_executor_->add_node(visualizer_node_);

    costmap_publisher_ = visualizer_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("simulation_costmap", rclcpp::QoS(10).reliable().transient_local());

    // Initialize tf buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(visualizer_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(visualizer_node_);

    for (auto &pair : *robotMaps_)
    {
        auto robot_name_callback = pair.first;
        if (movementMethod_ == "direct_jump")
        {
            auto nav2_action_server_ = rclcpp_action::create_server<NavigateToPose>(visualizer_node_, pair.first + "/navigate_to_pose",
                                                                                    std::bind(&RosInterface::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                                                    std::bind(&RosInterface::handle_cancel, this, std::placeholders::_1),
                                                                                    std::bind(&RosInterface::handle_accepted, this, std::placeholders::_1));
            nav2_action_server_vector_.push_back(nav2_action_server_);
        }
        else if (movementMethod_ == "cmd_vel_follow")
        {
            pair.second.lastCmdVelTime = std::make_shared<std::chrono::_V2::system_clock::time_point>(std::chrono::high_resolution_clock::now());
            // auto cmd_vel_subscriber = rclcpp::create_subscription<geometry_msgs::msg::Twist>(visualizer_node_, "/cmd_vel",
            //                                                                                 10, std::bind(&RosInterface::velocityCallback, this, std::placeholders::_1));
            auto cmd_vel_subscriber = rclcpp::create_subscription<geometry_msgs::msg::Twist>(
                visualizer_node_, pair.first + "/cmd_vel_nav", 2,
                [this, robot_name_callback](const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                    velocityCallback(msg, robot_name_callback);
                });
            cmd_vel_subscription_vector_.push_back(cmd_vel_subscriber);
        }

        auto pose_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(visualizer_node_, pair.first + "/execution_pose", 10);
        robotPublishers_[pair.first] = pose_publisher_;
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
    std::thread{std::bind(&RosInterface::executeDirectJump, this, std::placeholders::_1), goal_handle}.detach();
    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
}

void RosInterface::velocityCallback(geometry_msgs::msg::Twist::SharedPtr vel, std::string robotName)
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "Robot name: " << robotName);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Prev time: " << (*robotMaps_)[robotName].lastCmdVelTime);
    getRobotMutex()->lock();
    if ((*robotMaps_)[robotName].lastCmdVelTime == nullptr)
    {
        (*robotMaps_)[robotName].lastCmdVelTime = std::make_shared<std::chrono::_V2::system_clock::time_point>(std::chrono::high_resolution_clock::now());
        getRobotMutex()->unlock();
        return;
    }
    auto current_time_ = std::make_shared<std::chrono::_V2::system_clock::time_point>(std::chrono::high_resolution_clock::now());
    double elapsed_time = (*current_time_ - *(*robotMaps_)[robotName].lastCmdVelTime).count() * 1e-9;
    // discard stray values. Continous input stream of velocities of greater than 5Hz is needed.
    if (elapsed_time > 0.30)
    {
        (*robotMaps_)[robotName].lastCmdVelTime = nullptr;
        current_time_ = nullptr;
        getRobotMutex()->unlock();
        return;
    }
    (*robotMaps_)[robotName].lastCmdVelTime = current_time_;
    double vx = vel->linear.x;
    double vtheta = vel->angular.z;
    (*robotMaps_)[robotName].setPos.x = (*robotMaps_)[robotName].currentPos.x + (elapsed_time * vx) * cos((*robotMaps_)[robotName].currentPos.yaw);
    (*robotMaps_)[robotName].setPos.y = (*robotMaps_)[robotName].currentPos.y + (elapsed_time * vx) * sin((*robotMaps_)[robotName].currentPos.yaw);
    (*robotMaps_)[robotName].setPos.yaw = (*robotMaps_)[robotName].currentPos.yaw + (elapsed_time * vtheta);
    getRobotMutex()->unlock();

    // RCLCPP_INFO_STREAM(this->get_logger(), "Time: " << elapsed_time);
}

void RosInterface::executeDirectJump(const std::shared_ptr<GoalHandleNav2> goal_handle)
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
    auto r_pose = goal_handle->get_goal()->pose;
    r_pose.header.frame_id = "map";
    robotPublishers_[goal_handle->get_goal()->behavior_tree]->publish(r_pose);
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
        auto currentTime = visualizer_node_->now();
        transformStamped.header.stamp = visualizer_node_->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = pair.first + "/odom"; // Adjust child frame id as needed
        // Set the transformation values
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;

        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        transformStamped.transform.rotation = quaternion;

        // Publish the transformation
        tf_broadcaster_->sendTransform(transformStamped);
    }

    for (const auto &pair : *robotMaps_)
    {
        // RCLCPP_INFO(visualizer_node_->get_logger(), "Robot Name: %s, x: %f, y: %f, yaw: %f",
        //             pair.first.c_str(), pair.second.currentPos.x, pair.second.currentPos.y, pair.second.currentPos.yaw);
        // Publish transformation between frames
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = visualizer_node_->now();
        transformStamped.header.frame_id = pair.first + "/odom";
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