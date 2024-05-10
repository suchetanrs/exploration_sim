#include "exploration_sim/helpers.hpp"

// ------------------------ROS HELPERS-----------------------
void imageToOccupancy(nav_msgs::msg::OccupancyGrid & msg, double resolution, cv::Mat & resizedImage)
{
    msg.header              = msg.header;
    msg.info.height = resizedImage.rows;
    msg.info.width = resizedImage.cols;
    msg.info.resolution = resolution;
    msg.info.origin.position.x = 0.0;
    msg.info.origin.position.y = 0.0;
    msg.data.resize(msg.info.height * msg.info.width);

    // std::cout << "------------------" << std::endl;
    for (int y = 0; y < resizedImage.rows; ++y)
    {
        for (int x = 0; x < resizedImage.cols; ++x)
        {
            if (static_cast<int>(resizedImage.at<uint8_t>(y, x)) == 255)
                msg.data[x + y * msg.info.width] = -1;
            else if (static_cast<int>(resizedImage.at<uint8_t>(y, x)) < 255 && static_cast<int>(resizedImage.at<uint8_t>(y, x)) > 128)
                msg.data[x + y * msg.info.width] = 100;
            else
                msg.data[x + y * msg.info.width] = 0;
            // std::cout << "Value: " << static_cast<int>(resizedImage.at<uint8_t>(y, x)) << std::endl;
            // if (msg.data[x + y * msg.info.width] > 100)
            // {
            //     throw std::runtime_error("Value more than 100!");
            // }
        }
    }
}

bool pointInPolygon(const Point & point, const std::vector<Point> & polygon)
{
    int cross        = 0;
    auto polygonSize = polygon.size();
    Point polygonI;
    Point polygonJ;
    for (size_t i = 0, j = polygon.size() - 1; i < polygonSize; j = i++)
    {
        polygonI = polygon[i];
        polygonJ = polygon[j];
        if (
          ((polygonI.y > point.y) != (polygonJ.y > point.y)) &&
          (point.x < (polygonJ.x - polygonI.x) * (point.y - polygonI.y) / (polygonJ.y - polygonI.y) + polygonI.x))
        {
            cross++;
        }
    }
    return bool(cross % 2);
};