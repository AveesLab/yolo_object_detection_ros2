#ifndef YOLO_OBJECT_DETECTION_NODE_HPP
#define YOLO_OBJECT_DETECTION_NODE_HPP

//C++
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include <string>

#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "yolo_v2_class.hpp"


//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/boundingbox.hpp"
#include "ros2_msg/msg/yoloflag.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace yolo_object_detection_ros2 {

class YoloObjectDetectionNode : public rclcpp::Node
{
public:
    explicit YoloObjectDetectionNode();
    ~YoloObjectDetectionNode();

private:
    bool readParameters();
    void init();
    static std::vector<std::string> objectNames(std::string const filename);
    void runYoloCallback(const ros2_msg::msg::Yoloflag::SharedPtr msg);
    void rearCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void frontCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishInThread(std::vector<bbox_t> objects, std::string obj_name);
    void drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects);
    void recordData(struct timeval startTime);
    void detectInThread();

    /* ROS2 sub */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rearCamImgSubscriber_;    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frontCamImgSubscriber_;    
    rclcpp::Subscription<ros2_msg::msg::Yoloflag>::SharedPtr runYoloSubscriber_;
    
    /* pub */
    rclcpp::Publisher<ros2_msg::msg::Boundingbox>::SharedPtr boundingBoxPublisher_;

    cv::Mat rearCamImageCopy_;
    cv::Mat frontCamImageCopy_;
    bool imageStatus_ = false;

    std::thread detectThread_;
    std::mutex rear_cam_mutex_, front_cam_mutex_;

    Detector *yoloDetector_; // use smart ptr instead
    std::vector<std::string> objectNames_;
    std::vector<bbox_t> objects_;

    std::string names_;
    std::string cfg_;
    std::string weights_;

    bool f_run_yolo_ = false;
    bool r_run_yolo_ = false;
    bool viewImage_;
    bool enableConsoleOutput_;
    int waitKeyDelay_;

    int width_;
    int height_;

    struct timeval startTime_;
    double delay_ = 0.0;

    double sec_ = 0.0;
    double nsec_ = 0.0;
};

} // namespace yolo_object_detection

#endif // YOLO_OBJECT_DETECTION_NODE_HPP