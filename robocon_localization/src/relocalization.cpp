#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "distance_lookup.h"
#include "lines_matcher.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;

static DistanceLookup distanceLookup;
static float counter_x = 0;
static float counter_y = 0;
static float counter_yaw = 0;
static cv::Mat field_image;
static cv::Mat monitor_image;
static cv::Mat image_lines_all;
static cv::Mat image_lines_map;
static cv::Mat image_red_map;
static cv::Mat image_blue_map;

static bool flag_relocalization = false;
static bool is_robot_tilted = false;

static ros::Publisher pose_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
    // 检查标记变量
    if (!flag_relocalization) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;
    Mat image_show = image_raw.clone();
    Mat image_scope = image_raw.clone();
    image_scope = Scalar(0, 0, 0);  // 将背景设置为黑色
    Mat image_lines = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_corrected = image_lines.clone();
    Mat image_red = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_blue = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_sideline_red = image_lines.clone();
    Mat image_sideline_blue = image_lines.clone();

    // 1. HSV空间的白色检测
    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    Mat mask_hsv;
    inRange(image_hsv, Scalar(0, 0, 210), Scalar(180, 30, 255), mask_hsv); 

    // 2. RGB空间的白色检测
    Mat mask_rgb;
    inRange(image_raw, Scalar(200, 200, 200), Scalar(255, 255, 255), mask_rgb);

    Mat image_white = mask_hsv & mask_rgb;  // 使用与运算,要求同时满足两个条件

    // 添加红色检测
    Mat mask_red1, mask_red2;
    // HSV空间中红色的两个范围
    inRange(image_hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask_red1);
    inRange(image_hsv, Scalar(160, 100, 100), Scalar(180, 255, 255), mask_red2);
    image_red = mask_red1 | mask_red2;  // 合并两个红色范围

    // 添加蓝色检测
    Mat mask_blue;
    inRange(image_hsv, Scalar(100, 100, 100), Scalar(124, 255, 255), mask_blue);  // HSV空间中的蓝色范围
    image_blue = mask_blue;

    int center_x = image_hsv.cols / 2;
    int center_y = image_hsv.rows / 2;

    // 用射线扫描白线、红线和蓝线图像
    for (int angle = 0; angle < 360; angle += 4) {
        double rad = angle * CV_PI / 180.0;
        unsigned char last_pixel_white = 0;
        unsigned char last_pixel_red = 0;
        unsigned char last_pixel_blue = 0;
        
        for (int length = 0; length < std::max(image_hsv.cols, image_hsv.rows); length++) {
            int x = static_cast<int>(center_x + length * cos(rad));
            int y = static_cast<int>(center_y + length * sin(rad));
            
            if (x >= 0 && x < image_white.cols && y >= 0 && y < image_white.rows) {
                // 检测白线边缘
                unsigned char pixel_white = image_white.at<uchar>(y, x);
                if (last_pixel_white == 255 && pixel_white != 255) 
                {
                    // 在 image_show 中画紫色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 0, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 0, 255), 1);
                    cv::circle(image_lines, Point(x,y), 2, cv::Scalar(255, 255, 255), -1);

                    // 畸变矫正
                    double dist = distanceLookup.getDistance(length)*20;
                    if(dist > 0 && dist < 150)
                    {
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        image_corrected.at<uchar>(y2, x2) = 255;
                    }
                }
                last_pixel_white = pixel_white;

                // 检测红线边缘
                unsigned char pixel_red = image_red.at<uchar>(y, x);
                if (last_pixel_red != 255 && pixel_red == 255)  // 修改这里的条件
                {
                    // 在 image_show 中画黄色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(0, 255, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(0, 255, 255), 1);

                    // 畸变矫正
                    double dist = distanceLookup.getDistance(length)*20;
                    if(dist > 0 && dist < 150)
                    {
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        image_sideline_red.at<uchar>(y2, x2) = 255;
                    }
                }
                last_pixel_red = pixel_red;

                // 检测蓝线边缘
                unsigned char pixel_blue = image_blue.at<uchar>(y, x);
                if (last_pixel_blue != 255 && pixel_blue == 255)
                {
                    // 在 image_show 中画浅蓝色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 255, 0), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 255, 0), 1);

                    // 畸变矫正
                    double dist = distanceLookup.getDistance(length)*20;
                    if(dist > 0 && dist < 150)
                    {
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        image_sideline_blue.at<uchar>(y2, x2) = 255;
                    }
                }
                last_pixel_blue = pixel_blue;
            }
        }
    }

    // 将 image_corrected 顺时针旋转90度
    cv::Mat rotated_image;
    cv::rotate(image_corrected, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 计算旋转后图像的中心点
    cv::Point2f img_center(rotated_image.cols/2.0f, rotated_image.rows/2.0f);
    
    // 缩放比例变量
    const float scale_factor_x = 2.4f;  // X方向缩放因子
    const float scale_factor_y = 2.3f;  // Y方向缩放因子
    // 计算横向偏移量，使两个图像中心对齐
    float x_offset = (image_lines_map.cols - rotated_image.cols) / 2.0f;
    // 计算纵向偏移量
    float y_offset = (image_lines_map.rows - rotated_image.rows) / 2.0f;

    // 从旋转后的图像中提取白色、红色和蓝色点
    std::vector<cv::Point2f> white_points;
    std::vector<cv::Point2f> red_points;
    std::vector<cv::Point2f> blue_points;

    // 用于计算平均位置的变量
    cv::Point2f red_avg(0.0f, 0.0f);
    cv::Point2f blue_avg(0.0f, 0.0f);

    // 旋转红线和蓝线图像
    cv::Mat rotated_red, rotated_blue;
    cv::rotate(image_sideline_red, rotated_red, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(image_sideline_blue, rotated_blue, cv::ROTATE_90_CLOCKWISE);

    for(int y = 0; y < rotated_image.rows; y++) {
        for(int x = 0; x < rotated_image.cols; x++) {
            // 处理白点
            if(rotated_image.at<uchar>(y,x) == 255) {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                white_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
            // 处理红点
            if(rotated_red.at<uchar>(y,x) == 255) {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                red_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
            // 处理蓝点
            if(rotated_blue.at<uchar>(y,x) == 255) {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                blue_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
        }
    }

    /*********************************************************************************************/
    // 匹配开始
    counter_x = 0;
    counter_y = 0;
    counter_yaw = -1;

    // 只有当红点和蓝点都存在时才计算角度
    if (!red_points.empty() && !blue_points.empty()) {
        red_avg = LinesMatcher::calculateLinePointsAverage(red_points, image_lines_map.size());
        blue_avg = LinesMatcher::calculateLinePointsAverage(blue_points, image_lines_map.size());
        
        // 计算从red_avg指向blue_avg的角度
        float dx = blue_avg.x - red_avg.x;
        float dy = blue_avg.y - red_avg.y;
        float angle = atan2(dy, dx) * 180 / CV_PI;
        
        // 计算红点和蓝点的连线的中心点坐标
        cv::Point2f line_center((red_avg.x + blue_avg.x) / 2, (red_avg.y + blue_avg.y) / 2);
        counter_yaw = -90 - angle;
    }
    
    // [1]先使用红蓝边线进行初次匹配
    cv::Point2f center(img_center.x + x_offset, img_center.y + y_offset);

    int last_max_sum = 0;
    while (true) {
        MatchResult match_result = LinesMatcher::findMatch(red_points, blue_points, 
                                                            image_red_map, image_blue_map,
                                                            center, counter_x, counter_y, counter_yaw);
        
        // 如果本次匹配结果没有改善，则退出循环
        if (match_result.max_sum <= last_max_sum) {
            break;
        }
        
        // 更新计数器和上一次的最大匹配值
        counter_x = match_result.best_dx;
        counter_y = match_result.best_dy;
        counter_yaw = match_result.best_angle;
        last_max_sum = match_result.max_sum;
    }

    float rad = (counter_yaw) * CV_PI / 180.0;
    
    // 如果红点和蓝点都存在，则进行范围统计
    if (!red_points.empty() && !blue_points.empty()) {
        // 初始化范围统计变量
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        // 先统计红点范围
        for(const auto& point : red_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            min_x = std::min(min_x, rotated_x);
            max_x = std::max(max_x, rotated_x);
            min_y = std::min(min_y, rotated_y);
            max_y = std::max(max_y, rotated_y);
        }

        // 再加上统计蓝点范围
        for(const auto& point : blue_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            min_x = std::min(min_x, rotated_x);
            max_x = std::max(max_x, rotated_x);
            min_y = std::min(min_y, rotated_y);
            max_y = std::max(max_y, rotated_y);
        }

        // printf("点的范围 - X: [%.2f, %.2f], Y: [%.2f, %.2f]\n", min_x, max_x, min_y, max_y);

        // 利用红蓝边界，剔除边界外的白点
        std::vector<cv::Point2f> filtered_white_points;
        for(const auto& point : white_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            // 只保留在范围内的点
            if (rotated_x >= min_x && rotated_x <= max_x && 
                rotated_y >= min_y && rotated_y <= max_y) {
                filtered_white_points.push_back(point);
            }
        }
        // 用过滤后的点替换原来的white_points
        white_points = filtered_white_points;
    }

    
    // [定位测试] 在多个地点使用白点进行场线匹配
    int index_x[] = {-150, 0, 150};
    int index_y[] = {-100, 0, 100};
    int index_yaw[] = { 0, 45, 90, 135, 180, 225, 270, 315};

    int best_x = 0;
    int best_y = 0;
    int best_yaw = 0;
    int best_sum = 0;

    for(int i = 0; i < 3; i++) {
        int test_x = index_x[i];
        for(int j = 0; j < 3; j++) {
            int test_y = index_y[j];
            for(int k = 0; k < 8; k++) {
                int test_yaw = counter_yaw;
                if(test_yaw == -1) {
                    test_yaw = index_yaw[k];
                }
                // 用测试值开始匹配
                int last_white_sum = 0;
                while (true) {
                    MatchResult white_match = LinesMatcher::refineMatchWithWhitePoints(
                        white_points,
                        image_lines_map,
                        center,
                        test_x,
                        test_y,
                        test_yaw
                    );
                    
                    // 如果本次匹配结果没有改善，则退出循环
                    if (white_match.max_sum <= last_white_sum) {
                        break;
                    }
                    
                    // 更新计数器和上一次的最大匹配值
                    test_x += white_match.best_dx;
                    test_y += white_match.best_dy;
                    test_yaw += white_match.best_angle;
                    last_white_sum = white_match.max_sum;
                }
                // 此时完成了一轮完整匹配，记录结果
                if(last_white_sum > best_sum) {
                    best_x = test_x;
                    best_y = test_y;
                    best_yaw = test_yaw;
                    best_sum = last_white_sum;
                }
            }
        }
    }

    // 所有测试完成，显示最佳结果
    counter_x = best_x;
    counter_y = best_y;
    counter_yaw = best_yaw;

    // 匹配结束
    /*********************************************************************************************/
    // 显示匹配效果
    cv::Mat image_match_result = image_lines_all.clone();
    // cv::Mat image_match_result = image_lines_map.clone();
    // cv::Mat image_match_result = image_red_map.clone();
    // cv::Mat image_match_result = cv::Mat::zeros(image_red_map.size(), CV_8UC1);
    // for(int y = 0; y < image_match_result.rows; y++) {
    //     for(int x = 0; x < image_match_result.cols; x++) {
    //         image_match_result.at<uchar>(y,x) = std::max(image_red_map.at<uchar>(y,x), 
    //                                               image_blue_map.at<uchar>(y,x));
    //     }
    // }
    // cv::cvtColor(image_match_result, image_match_result, cv::COLOR_GRAY2BGR);
    
    ROS_INFO("最佳匹配结果: counter_x: %.2f, counter_y: %.2f, counter_yaw: %.2f\n", counter_x, counter_y, counter_yaw);

    // 绘制白点
    for(const auto& point : white_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,255,0), -1);
        }
    }

    // 绘制红点
    for(const auto& point : red_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,0,255), -1);
        }
    }

    // 绘制蓝点
    for(const auto& point : blue_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(255,0,0), -1);
        }
    }
    
    // 将各种线条标记到 image_scope 上
    // for(int y = 0; y < image_scope.rows; y++) {
    //     for(int x = 0; x < image_scope.cols; x++) {
    //         // if(image_corrected.at<uchar>(y,x) == 255) {
    //         //     // 白色点表示白线
    //         //     circle(image_scope, Point(x,y), 2, Scalar(255,255,255), -1);
    //         // }
    //         if(image_sideline_red.at<uchar>(y,x) == 255) {
    //             // 红色点表示红线
    //             circle(image_scope, Point(x,y), 2, Scalar(0,0,255), -1);
    //         }
    //         if(image_sideline_blue.at<uchar>(y,x) == 255) {
    //             // 蓝色点表示蓝线
    //             circle(image_scope, Point(x,y), 2, Scalar(255,0,0), -1);
    //         }
    //     }
    // }

    // 在image_scope上绘制筛选后的white_points
    // for(const auto& point : white_points) {
    //     if(point.x >= 0 && point.x < image_scope.cols &&
    //        point.y >= 0 && point.y < image_scope.rows) {
    //         int x = -(point.x - x_offset - img_center.x)/scale_factor_x + img_center.x;
    //         int y = (point.y - y_offset - img_center.y)/scale_factor_y + img_center.y;
    //         circle(image_scope, Point(y, x), 2, Scalar(255,255,255), -1);
    //     }
    // }

    // 创建显示用的图像副本
    field_image.copyTo(monitor_image(cv::Rect(0, 50, field_image.cols, field_image.rows)));
    
    // 清除并更新信息区域
    cv::Mat info_area = monitor_image(cv::Rect(0, 0, monitor_image.cols, 50));
    info_area.setTo(cv::Scalar(255, 255, 255));  // 清除之前的文本
    
    std::string coordinates = cv::format("X: %.2f  Y: %.2f  Yaw: %.2f", counter_x, counter_y, counter_yaw);
    cv::putText(monitor_image, coordinates, cv::Point(200, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    
    // 绘制机器人图标
    // 计算field_image的中心点
    cv::Point2f bottom_center(field_image.cols/2.0f, field_image.rows/2.0f+50);
    
    // 计算机器人在图像中的位置
    float robot_x = bottom_center.x + counter_x * 1.0;
    float robot_y = bottom_center.y + counter_y * 1.0;
    cv::Point robot_pos(robot_x, robot_y);
    
    // 绘制机器人主体（紫色填充的圆形，黑色轮廓）
    int robot_radius = 15;
    cv::circle(monitor_image, robot_pos, robot_radius, cv::Scalar(0,0,0), 2);  // 黑色轮廓
    cv::circle(monitor_image, robot_pos, robot_radius-2, cv::Scalar(255,0,255), -1);  // 紫色填充
    
    // 绘制朝向线段
    float direction_length = 20.0f;
    float direction_rad = counter_yaw * CV_PI / 180.0;
    cv::Point direction_end(
        robot_x + direction_length * cos(direction_rad),
        robot_y + direction_length * sin(direction_rad)
    );
    cv::line(monitor_image, robot_pos, direction_end, cv::Scalar(0,0,0), 2);

    // 显示
    // cv::imshow("result", image_show);
    // cv::imshow("scope", image_scope);
    // cv::imshow("match_result", image_match_result);
    // cv::imshow("定位", monitor_image);
    // cv::waitKey(1);

    flag_relocalization = false;

    // 在匹配结果处理完成后，发布位置信息
    // 创建并发布位姿消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    
    // 设置位置 (单位转换: 像素->米)
    pose_msg.pose.position.x = counter_x / 100.0;  // 假设100像素=1米
    pose_msg.pose.position.y = counter_y / 100.0;
    pose_msg.pose.position.z = 0.0;
    
    // 设置方向（将角度转换为四元数）
    float yaw_rad = counter_yaw * M_PI / 180.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = sin(yaw_rad / 2.0);
    pose_msg.pose.orientation.w = cos(yaw_rad / 2.0);
    
    // 发布消息
    pose_pub.publish(pose_msg);
}

static const float TILT_THRESHOLD = 1.0f;  // 倾斜阈值（度）

// IMU回调函数
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    // 显示roll和pitch的角度值 0～360
    float roll = msg->orientation.x * 180 / CV_PI;
    float pitch = msg->orientation.y * 180 / CV_PI;

    // 使用绝对值判断倾斜状态
    if(std::abs(roll) > TILT_THRESHOLD || std::abs(pitch) > TILT_THRESHOLD) {
        is_robot_tilted = true;
        ROS_INFO("机器人倾倒 roll: %.2f 度, pitch: %.2f 度", roll, pitch);
    }

    // 当机器人从倾倒回复到正常状态时，开始进行重定位
    if(is_robot_tilted && std::abs(roll) < TILT_THRESHOLD && std::abs(pitch) < TILT_THRESHOLD) {
        flag_relocalization = true;
        is_robot_tilted = false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "relocalization");
    ros::NodeHandle nh("~");

    std::string table_file;
    nh.param<std::string>("table_file", table_file, "distances.txt");
    distanceLookup.init(table_file);

    std::string lines_file;
    nh.param<std::string>("lines_file", lines_file, "lines.jpg");
    image_lines_all = cv::imread(lines_file);

    std::string field_file;
    nh.param<std::string>("field_file", field_file, "field_bg.png");
    cv::Mat resultImage = cv::imread(field_file);
    if(resultImage.empty()) {
        ROS_ERROR("读取场地背景图失败: %s", field_file.c_str());
        return -1;
    }
    
    
    // 初始化field_image为彩色图像，尺寸为resultImage的一半
    cv::resize(resultImage, field_image, cv::Size(resultImage.cols/2, resultImage.rows/2));
    
    // 初始化monitor_image，大小与field_image相同
    monitor_image = cv::Mat(field_image.rows + 50, field_image.cols, CV_8UC3);

    
    // 读取参数
    std::string lines_map_path, red_map_path, blue_map_path;
    nh.param<std::string>("lines_map_file", lines_map_path, "");
    nh.param<std::string>("red_map_file", red_map_path, "");
    nh.param<std::string>("blue_map_file", blue_map_path, "");
    // 读取图像为灰度图
    image_lines_map = cv::imread(lines_map_path, cv::IMREAD_GRAYSCALE);
    image_red_map = cv::imread(red_map_path, cv::IMREAD_GRAYSCALE);
    image_blue_map = cv::imread(blue_map_path, cv::IMREAD_GRAYSCALE);

    // 检查图像是否成功加载
    if(image_lines_map.empty() || image_red_map.empty() || image_blue_map.empty()) {
        ROS_ERROR("场线模板读取失败！");
        return -1;
    }

    // 在订阅器之前添加发布器初始化
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/reloc_pose", 1);

    // 添加IMU订阅器
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 1, imuCallback);
    ros::Subscriber sub = nh.subscribe("/omni_camera/image_raw", 1, imageCallback);

    ros::spin();

    return 0;
}
