#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Publisher image_pub;	// Publisher defined globally

void publishImage(cv::Mat& image, ros::Publisher& publisher)	// Funciton to publish the image
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();	// Convert the OpenCV image to a ROS sensor_msgs::Image message
    publisher.publish(msg);	//Publishes the image message
    ros::Duration(1.0).sleep(); // Wait for 2 seconds
}

void drawLine(cv::Mat& image, int x0, int y0, int x1, int y1)	// Function to shoot an arrow from start to goal
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (x0 != x1 || y0 != y1)
    {
        if (x0 >= 0 && y0 >= 0 && x0 < image.cols && y0 < image.rows)	// to check the current pixel
        {
            if (image.at<cv::Vec3b>(x0, y0) == cv::Vec3b(0, 0, 0))	// check if current pixel is obstacle or not
            {
                ROS_INFO_STREAM("Obstacle encountered at (" << x0 << ", " << y0 << "), stopping.");
                break; // Stop drawing if obstacle encountered
            }
            image.at<cv::Vec3b>(x0, y0) = cv::Vec3b(0, 255, 0); // Green colour
            publishImage(image, image_pub);
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    // Marking the goal point
    if (x1 >= 0 && y1 >= 0 && x1 < image.cols && y1 < image.rows)
    {   
        if (image.at<cv::Vec3b>(x0, y0) == cv::Vec3b(0, 0, 0))
            {
                ROS_INFO_STREAM("Obstacle encountered at (" << x0 << ", " << y0 << "), stopping.");
                return; // Stop drawing if obstacle encountered
            }
        image.at<cv::Vec3b>(x1, y1) = cv::Vec3b(0, 255, 0); // Green color
        publishImage(image, image_pub);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");	// Initialized the ROS node
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");	// Private node handler
    
    // Reading the width and heigth of the image
    int width, height;
    nh_priv.getParam("width", width);
    nh_priv.getParam("height", height);
    ROS_INFO_STREAM(width);
    ROS_INFO_STREAM(height);
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    image_pub = nh.advertise<sensor_msgs::Image>("/image", 1);
    publishImage(image, image_pub);
    
    // Reading the start and goal points
    std::vector<int> start, goal;
    nh_priv.getParam("start", start);
    ROS_INFO_STREAM(start[0]);
    ROS_INFO_STREAM(start[1]);
    if (start[0] < 0 || start[0] >= height || start[1] < 0 || start[1] >= width)
    {
        ROS_ERROR_STREAM("Start point is out of bounds.");
        return -1;
    }
    image.at<cv::Vec3b>(start[0], start[1]) = cv::Vec3b(0, 255, 255);	// Yellow colour
    publishImage(image, image_pub);

    nh_priv.getParam("goal", goal);
    ROS_INFO_STREAM(goal[0]);
    ROS_INFO_STREAM(goal[1]);
    if (goal[0] < 0 || goal[0] >= height || goal[1] < 0 || goal[1] >= width)
    {
        ROS_ERROR_STREAM("Goal point is out of bounds.");
        return -1;
    }
    image.at<cv::Vec3b>(goal[0], goal[1]) = cv::Vec3b(255, 0, 0);	// Blue colour
    publishImage(image, image_pub);
    
    // Reading the obstacles points
    XmlRpc::XmlRpcValue obstacles;
    nh_priv.getParam("obstacles", obstacles);
    for (int i = 0; i < obstacles.size(); i++)
    {
        int obstacle_x = static_cast<int>(obstacles[i][0]);
        int obstacle_y = static_cast<int>(obstacles[i][1]);
        ROS_INFO_STREAM(obstacle_x);
        ROS_INFO_STREAM(obstacle_y);
        image.at<cv::Vec3b>(obstacle_x, obstacle_y) = cv::Vec3b(0, 0, 0);
        publishImage(image, image_pub);
    }
    
    // Function call to Draw the straight line from start to goal point
    drawLine(image, start[0], start[1], goal[0], goal[1]);

    return 0;
}
