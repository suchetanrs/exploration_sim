#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

struct Pose2D
{
    double x;
    double y;
    double yaw;

    Pose2D() {};

    // Operator!=
    bool operator!=(const Pose2D& other) const {
        // Compare the member variables of two instances
        return (this->x != other.x) ||
               (this->y != other.y) ||
               (this->yaw != other.yaw);
    }
};

struct RobotCharacteristics
{
    double FOV;
    Pose2D currentPos;
    Pose2D setPos;

    RobotCharacteristics() {};
};

struct Point
{
    double x;
    double y;
};

// ------------------------ROS HELPERS-----------------------
// TODO: suchetan - move this to a seperate file later.

void imageToOccupancy(nav_msgs::msg::OccupancyGrid & msg, double resolution, cv::Mat & resizedImage);

bool pointInPolygon(const Point & point, const std::vector<Point> & polygon);