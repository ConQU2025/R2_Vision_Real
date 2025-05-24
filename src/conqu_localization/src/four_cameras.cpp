#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;
Mat fused_image(480, 2560, CV_8UC3, cv::Scalar(0, 0, 0)); // 融合后的图片

void imageFusion(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;

    if (msg->header.frame_id == "usb_cam0")
    {
        image_raw.copyTo(fused_image(cv::Rect(0, 0, image_raw.cols, image_raw.rows)));
    }
    else if (msg->header.frame_id == "usb_cam1")
    {
        image_raw.copyTo(fused_image(cv::Rect(640, 0, image_raw.cols, image_raw.rows)));
    }
    else if (msg->header.frame_id == "usb_cam2")
    {
        image_raw.copyTo(fused_image(cv::Rect(1280, 0, image_raw.cols, image_raw.rows)));
    }
    else if (msg->header.frame_id == "usb_cam3")
    {
        image_raw.copyTo(fused_image(cv::Rect(1920, 0, image_raw.cols, image_raw.rows)));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "four_cameras");
    ros::NodeHandle nh("~");

    // 创建OpenCV窗口
    cv::namedWindow("Fused Image", cv::WINDOW_AUTOSIZE);
    
    ros::Subscriber sub0 = nh.subscribe("/usb_cam0/image_raw", 1, imageFusion);
    ros::Subscriber sub1 = nh.subscribe("/usb_cam1/image_raw", 1, imageFusion);
    ros::Subscriber sub2 = nh.subscribe("/usb_cam2/image_raw", 1, imageFusion);
    // ros::Subscriber sub3 = nh.subscribe("/usb_cam3/image_raw", 1, imageFusion);

    ROS_INFO("Four cameras node started, waiting for images...");
    
    // 使用自定义的spin循环
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        cv::imshow("Fused Image", fused_image);
        cv::waitKey(1);
        rate.sleep();
    }
    
    cv::destroyAllWindows();
    return 0;
}
