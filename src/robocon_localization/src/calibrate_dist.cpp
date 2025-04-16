#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;

    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    int center_x = image_hsv.cols / 2;
    int center_y = image_hsv.rows / 2;
    int distance = -1;

    Mat image_show = image_raw.clone(); // Create a copy of the raw image for display

    int x = center_x;
    for (int y = center_y; y >= 0; y--) {
        Vec3b pixel = image_hsv.at<Vec3b>(y, x);
        // Check for red color in HSV
        if (pixel[0] >= 0 && pixel[0] <= 10 && pixel[1] >= 100 && pixel[1] <= 255 && pixel[2] >= 100) {
            distance = center_y - y; // Calculate distance in pixels
            
            // Draw a yellow cross at the detected pixel
            int cross_size = 10; // Size of the cross
            line(image_show, Point(x - cross_size, y), Point(x + cross_size, y), Scalar(0, 255, 255), 2); // Horizontal line
            line(image_show, Point(x, y - cross_size), Point(x, y + cross_size), Scalar(0, 255, 255), 2); // Vertical line
            break;
        }
        // Mark the pixel in the image_show as green
        image_show.at<Vec3b>(y, x) = Vec3b(0, 255, 0); // Green color in BGR
    }

    if (distance != -1) {
        ROS_INFO("红球像素距离: %d pixels", distance);

    } else {
        ROS_INFO("未扫描到红色");
    }

    // Display the image in a window
    cv::imshow("result", image_show);
    cv::waitKey(30);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibrate_dist");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/omni_camera/image_raw", 1, imageCallback);

    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
    ros::spin();

    cv::destroyWindow("result");
    return 0;
}
