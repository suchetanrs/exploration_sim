#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "exploration_sim/rosInterface.hpp"

class ExplorationSim
{
public:
    ExplorationSim(const std::string &filename)
    {
        origin_x_ = 0.0;
        origin_y_ = 0.0;
        resolution_ = 0.05;
        // Read the image_
        image_ = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        // sanity checks
        if (image_.empty())
        {
            std::cerr << "Error: Unable to read image_ " << filename << std::endl;
        }
        robotMaps = std::make_shared<std::map<std::string, RobotCharacteristics>>();
        exploredImage_ = cv::Mat(image_.size(), image_.type(), cv::Scalar(255));
        makeRobot("/scout_1", 2.5);
        makeRobot("/scout_2", 2.5);
        rosViz_ = std::make_shared<RosInterface>(robotMaps);
        Pose2D pose;
        pose.x = 5.5;
        pose.y = 5.5;
        pose.yaw = CV_PI / 6;
        exploreMapRayTrace("/scout_1", pose);
        pose.x = 11.5;
        pose.y = 5.5;
        pose.yaw = CV_PI / 3;
        exploreMapRayTrace("/scout_2", pose);
        std::thread t(&ExplorationSim::exploreMapLoop, this);
        t.detach();
    }

    void makeRobot(std::string robotName, double FOV)
    {
        RobotCharacteristics robotChar;
        robotChar.FOV = FOV;
        (*robotMaps)[robotName] = robotChar;
    }

    void exploreMapLoop()
    {
        while (1 && rclcpp::ok())
        {
            rosViz_->getRobotMutex()->lock();
            for (auto &pair : *robotMaps)
            {
                if (pair.second.currentPos != pair.second.setPos)
                    exploreMapRayTrace(pair.first, pair.second.setPos);
            }
            rosViz_->getRobotMutex()->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void exploreMap(std::string robotName, Pose2D &pose)
    {
        (*robotMaps)[robotName].setPos = pose;
        std::vector<Point> vertices;
        // how much to move the pose from the vertex.
        double offset = 0.5;
        Point p1;
        p1.x = pose.x - offset * std::cos(pose.yaw);
        p1.y = pose.y - offset * std::sin(pose.yaw);
        vertices.push_back(p1);

        Point p2;
        p2.x = p1.x + (*robotMaps)[robotName].FOV * std::cos(pose.yaw - CV_PI / 6);
        p2.y = p1.y + (*robotMaps)[robotName].FOV * std::sin(pose.yaw - CV_PI / 6);
        vertices.push_back(p2);

        Point p3;
        p3.x = p1.x + (*robotMaps)[robotName].FOV * std::cos(pose.yaw + CV_PI / 6);
        p3.y = p1.y + (*robotMaps)[robotName].FOV * std::sin(pose.yaw + CV_PI / 6);
        vertices.push_back(p3);
        auto w_centroid_x = (p1.x + p2.x + p3.x) / 3;
        auto w_centroid_y = (p1.y + p2.y + p3.y) / 3;
        int m_centroid_x = 0;
        int m_centroid_y = 0;
        int fovPixels = sizeWorldToMap((*robotMaps)[robotName].FOV);
        worldToMapNoBool(m_centroid_x, m_centroid_y, w_centroid_x, w_centroid_y);

        // Define vertices of the triangle
        for (int y = m_centroid_y - (fovPixels * 2); y < m_centroid_y + (fovPixels * 2); ++y)
        {
            for (int x = m_centroid_x - (fovPixels * 2); x < m_centroid_x + (fovPixels * 2); ++x)
            {
                if (x < 0 || y < 0 || y >= image_.rows || x >= image_.cols)
                {
                    // std::cout << "Out of boundaries: " << x << " y: " << y << std::endl;
                    continue;
                }
                // Check if the pixel value is equal to oldValue
                Point interest;
                mapToWorld(x, y, interest.x, interest.y);
                if (pointInPolygon(interest, vertices))
                {
                    exploredImage_.at<uint8_t>(y, x) = image_.at<uint8_t>(y, x);
                }
            }
        }
        // Define vertices of the triangle
        // for (int y = 0; y < image_.rows; ++y)
        // {
        //     for (int x = 0; x < image_.cols; ++x)
        //     {
        //         // Check if the pixel value is equal to oldValue
        //         Point interest;
        //         mapToWorld(x, y, interest.x, interest.y);
        //         if (pointInPolygon(interest, vertices))
        //         {
        //             exploredImage_.at<uint8_t>(y, x) = image_.at<uint8_t>(y, x);
        //         }
        //     }
        // }
        // set currentPos to setPos.
        rosViz_->publishCostmap(exploredImage_);
        (*robotMaps)[robotName].currentPos = pose;
    }

    void exploreMapRayTrace(std::string robotName, Pose2D &pose)
    {
        (*robotMaps)[robotName].setPos = pose;
        int fovWorld = (*robotMaps)[robotName].FOV;
        double rayStep = 0.01; // Step size for the ray tracing
        // int numRays = 22;      // enable if you want realistic exploration
        int numRays = 50;      // enable if you want solid colour

        double offset = 0.5;
        Point startPoint;
        startPoint.x = pose.x - offset * std::cos(pose.yaw);
        startPoint.y = pose.y - offset * std::sin(pose.yaw);

        for (int i = 0; i < numRays; ++i)
        {
            auto breakFlag = false;
            double angle = pose.yaw - (CV_PI / 6) + (i * CV_PI / (numRays * 3));
            for (double r = 0; r < fovWorld; r += rayStep)
            {
                if (breakFlag)
                {
                    breakFlag = false;
                    break;
                }
                Point currentPoint;
                currentPoint.x = startPoint.x + r * std::cos(angle);
                currentPoint.y = startPoint.y + r * std::sin(angle);

                int mapX, mapY;
                worldToMapNoBool(mapX, mapY, currentPoint.x, currentPoint.y);

                if (mapX < 0 || mapY < 0 || mapX >= image_.cols || mapY >= image_.rows)
                    break; // Break if the ray goes out of bounds

                if (image_.at<uint8_t>(mapY, mapX) > 250)
                    breakFlag = true;

                exploredImage_.at<uint8_t>(mapY, mapX) = image_.at<uint8_t>(mapY, mapX);
            }
        }
        rosViz_->publishCostmap(exploredImage_);
        (*robotMaps)[robotName].currentPos = pose;
    }

    void displayImage()
    {
        // Display the image_
        // cv::imshow("Exploration Simulation", exploredImage_);
        // cv::waitKey(0);          // Wait for a key press
        // cv::destroyAllWindows(); // Close all OpenCV windows
        // Display the image_
        // cv::imshow("Exploration Simulation", image_);
        // cv::waitKey(0);          // Wait for a key press
        // cv::destroyAllWindows(); // Close all OpenCV windows
        rosViz_->sendROSTransform();
        while (1 && rclcpp::ok())
        {
            rosViz_->publishCostmap(exploredImage_);
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    }

private:
    cv::Mat image_;
    cv::Mat exploredImage_;
    double origin_x_;
    double origin_y_;
    double resolution_;
    std::shared_ptr<RosInterface> rosViz_;

    int numRobots = 2;
    std::shared_ptr<std::map<std::string, RobotCharacteristics>> robotMaps;

    void mapToWorld(int map_x, int map_y, double &world_x, double &world_y) const
    {
        // Convert map coordinates to world coordinates
        world_x = origin_x_ + (map_x + 0.5) * resolution_;
        world_y = origin_y_ + (map_y + 0.5) * resolution_;
    }

    bool worldToMap(unsigned int &map_x, unsigned int &map_y, double world_x, double world_y) const
    {
        if (world_x < origin_x_ || world_y < origin_y_)
        {
            return false;
        }

        map_x = static_cast<unsigned int>((world_x - origin_x_) / resolution_);
        map_y = static_cast<unsigned int>((world_y - origin_y_) / resolution_);

        if (map_x < image_.cols && map_y < image_.rows)
        {
            return true;
        }
        return false;
    }

    void worldToMapNoBool(int &map_x, int &map_y, double world_x, double world_y) const
    {
        map_x = static_cast<int>((world_x - origin_x_) / resolution_);
        map_y = static_cast<int>((world_y - origin_y_) / resolution_);
    }

    double sizeMapToWorld(int size)
    {
        return size * resolution_;
    }

    int sizeWorldToMap(double size)
    {
        return static_cast<int>(size / resolution_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Provide the filename of the image_ in the program's path
    std::string filename = "/root/dev_ws/src/exploration_sim/src/map_05res_ros.jpg";

    ExplorationSim explorer(filename);
    explorer.displayImage();

    return 0;
}