#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "distance_lookup.h"
#include "lines_map.h"
#include "lines_matcher.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;

static DistanceLookup distanceLookup;
static LinesMap lines_map;
static float counter_x = 0;
static float counter_y = 0;
static float counter_yaw = 0;
static cv::Mat field_image, monitor_image, image_red_map, image_blue_map, image_lines_all, image_lines_map;

const cv::Vec3b COLORS[4] = {
    cv::Vec3b(255, 255, 255), // 白
    cv::Vec3b(0, 0, 255), // 红
    cv::Vec3b(255, 0, 0), // 蓝
    cv::Vec3b(0, 0, 0), // 灰
};


// S值小于GRAY_S_THREHOLD的像素被认为是灰色
// 灰色像素中，V值大于WHITE_V_THREHOLD的像素被认为是白色
// 剩余像素H值若属于BLUE_H_MIN和BLUE_H_MAX之间，则是蓝色，否则是红色（CV的H值范围是0~179）
int blue_h_max, blue_h_min, gray_s_threhold, white_v_min, white_v_max, red_h_max, red_h_min, gray_v_threhold;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
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

    // 1. 提高对比度（CLAHE）
    // cv::Mat lab_image;
    // cv::cvtColor(image_raw, lab_image, cv::COLOR_BGR2Lab);
    // std::vector<cv::Mat> lab_planes(3);
    // cv::split(lab_image, lab_planes);
    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8,8));
    // clahe->apply(lab_planes[0], lab_planes[0]);
    // cv::merge(lab_planes, lab_image);
    // cv::cvtColor(lab_image, image_raw, cv::COLOR_Lab2BGR);

    // 2. 去除反光（使用大津阈值法处理高亮区域）
    cv::Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_planes;
    cv::split(image_hsv, hsv_planes);
    
    // 使用大津阈值法自动确定阈值
    cv::Mat mask_reflect;
    double otsu_thresh = cv::threshold(
        hsv_planes[2], mask_reflect, 0, 255, 
        cv::THRESH_BINARY | cv::THRESH_OTSU
    );
    
    // 输出当前使用的大津阈值
    ROS_INFO("当前大津阈值: %.2f", otsu_thresh);
    
    // 将高亮区域V值降低到适中的值
    hsv_planes[2].setTo(200, mask_reflect); // 将高亮区域V值降为200
    
    // 合并通道并转换回BGR
    cv::merge(hsv_planes, image_hsv);
    cv::cvtColor(image_hsv, image_raw, cv::COLOR_HSV2BGR);


    // 初始化掩码图像和其他图像
    Mat image_show = image_raw.clone();                               // 用于显示原始量化结果
    Mat image_scope = Mat::zeros(image_raw.size(), image_raw.type()); // 全黑背景
    Mat image_lines = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_corrected = image_lines.clone();
    Mat image_sideline_red = image_lines.clone();
    Mat image_sideline_blue = image_lines.clone();

    int center_x = image_raw.cols / 2;
    int center_y = image_raw.rows / 2;
    // int max_length = std::max(image_raw.cols, image_raw.rows);

    // cv::Mat hsl_img;
    // cv::cvtColor(image_raw, hsl_img, cv::COLOR_BGR2HLS);
    // 用射线扫描图像
    for (int angle = 0; angle < 360; angle += 1)
    { 
        double rad = angle * CV_PI / 180.0;
        double cos_rad = cos(rad);
        double sin_rad = sin(rad);
        unsigned char last_pixel = 3;

        for (int length = 0; length < image_raw.cols / 2; length++)
        {
            int x = static_cast<int>(center_x + length * cos_rad);
            int y = static_cast<int>(center_y + length * sin_rad);

            // 检查坐标    setlocale(LC_ALL, "");
    ros::init(argc, argv, "loc_sidelines");
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
            // 将图像的每个像素转换为红、蓝、白、灰中最近的颜色
            // cv::Vec3b pix = image_raw.at<cv::Vec3b>(y, x);
            // int min_idx = 0;
            // int min_dist = INT_MAX;
            // for (int i = 0; i < 4; ++i)
            // {
            //     int db = int(pix[0]) - int(COLORS[i][0]);
            //     int dg = int(pix[1]) - int(COLORS[i][1]);
            //     int dr = int(pix[2]) - int(COLORS[i][2]);
            //     int dist = db * db + dg * dg + dr * dr;
            //     if (dist < min_dist)
            //     {
            //         min_dist = dist;
            //         min_idx = i;
            //     }
            // }
            int min_idx = 0;
            cv::Vec3b pix = image_hsv.at<cv::Vec3b>(y, x);

            if (pix[2] < gray_v_threhold){
                min_idx = 3;
            }
            else if (pix[1] < gray_s_threhold) // S值小于阈值，认为是灰色
            {
                if (pix[2] >= white_v_min && pix[2] <= white_v_max){ // 白色
                    min_idx = 0;
                }
                else // 否则认为是灰色
                {
                    min_idx = 3;
                }    setlocale(LC_ALL, "");
    ros::init(argc, argv, "loc_sidelines");
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
                    min_idx = 3; // 其他颜色
                }
            }

            // 更新量化后的图像 (用于 image_show)
            image_show.at<cv::Vec3b>(y, x) = COLORS[min_idx];

            // 根据最接近的颜色索引，查找对应的边缘
            double dist = distanceLookup.getDistance(length) * 20;
            int x2 = static_cast<int>(center_x + dist * cos_rad);
            int y2 = static_cast<int>(center_y + dist * sin_rad);

            // 畸变校正后记录边缘
            if (last_pixel != min_idx && (dist > 0 && dist < 150) && (x2 >= 0 && x2 < image_raw.cols && y2 >= 0 && y2 < image_raw.rows))
            {
                switch (min_idx)
                {
                case 0: // White
                    // 在 image_show 中画紫色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 0, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 0, 255), 1);
                    cv::circle(image_lines, Point(x, y), 2, cv::Scalar(255, 255, 255), -1);

                    image_corrected.at<uchar>(y2, x2) = 255;
                    break;
                case 1: // Red
                    // 在 image_show 中画黄色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(0, 255, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(0, 255, 255), 1);

                    im    setlocale(LC_ALL, "");
    ros::init(argc, argv, "loc_sidelines");
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
    }age_sideline_red.at<uchar>(y2, x2) = 255;
                    break;
                case 2: // Blue
                    // 在 image_show 中画浅蓝色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 255, 0), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 255, 0), 1);

                    image_sideline_blue.at<uchar>(y2, x2) = 255;
                    break;
                default:
                    break;
                }
            }
            last_pixel = min_idx;
        }
    }

    // 将 image_corrected 顺时针旋转90度
    cv::Mat rotated_image;
    cv::rotate(image_corrected, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 计算旋转后图像的中心点
    cv::Point2f img_center(rotated_image.cols / 2.0f, rotated_image.rows / 2.0f);

    // 缩放比例变量
    // TODO: 调参
    const float scale_factor_x = 2.4f; // X方向缩放因子
    const float scale_factor_y = 2.3f; // Y方向缩放因子
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

    for (int y = 0; y < rotated_image.rows; y++)
    {
        for (int x = 0; x < rotated_image.cols; x++)
        {
            // 处理白点
            if (rotated_image.at<uchar>(y, x) == 255)
            {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                white_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
            // 处理红点
            if (rotated_red.at<uchar>(y, x) == 255)
            {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                red_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
            // 处理蓝点
            if (rotated_blue.at<uchar>(y, x) == 255)
            {
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor_x + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor_y + y_offset;
                blue_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
        }
    }

    // [1]先使用红蓝边线进行初次匹配
    red_avg = LinesMatcher::calculateLinePointsAverage(red_points, image_lines_map.size());
    blue_avg = LinesMatcher::calculateLinePointsAverage(blue_points, image_lines_map.size());

    // 计算从red_avg指向blue_avg的角度
    float dx = blue_avg.x - red_avg.x;
    float dy = blue_avg.y - red_avg.y;
    float angle = atan2(dy, dx) * 180 / CV_PI;

    // 计算红点和蓝点的连线的中心点坐标
    cv::Point2f line_center((red_avg.x + blue_avg.x) / 2, (red_avg.y + blue_avg.y) / 2);
    counter_yaw = -90 - angle;

    cv::Point2f center(img_center.x + x_offset, img_center.y + y_offset);

    int last_max_sum = 0;
    while (true)
    {
        MatchResult match_result = LinesMatcher::findMatch(
            red_points, blue_points,
            image_red_map, image_blue_map,
            center, counter_x, counter_y, counter_yaw);

        // 如果本次匹配结果没有改善，则退出循环
        if (match_result.max_sum <= last_max_sum)
        {
            break;
        }

        // 更新计数器和上一次的最大匹配值
        counter_x += match_result.best_dx;
        counter_y += match_result.best_dy;
        counter_yaw += match_result.best_angle;
        last_max_sum = match_result.max_sum;
    }

    float rad = (counter_yaw)*CV_PI / 180.0;

    // 初始化范围统计变量
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    // 合并红点和蓝点以进行范围计算
    std::vector<cv::Point2f> boundary_points = red_points;
    boundary_points.insert(boundary_points.end(), blue_points.begin(), blue_points.end());

    // 统计红点和蓝点的范围
    for (const auto &point : boundary_points)
    {
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
    for (const auto &point : white_points)
    {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        // 只保留在范围内的点
        if (rotated_x >= min_x && rotated_x <= max_x &&
            rotated_y >= min_y && rotated_y <= max_y)
        {
            filtered_white_points.push_back(point);
        }
    }
    // 用过滤后的点替换原来的white_points
    white_points = filtered_white_points;

    // [2] 使用白点进行场线匹配
    int last_white_sum = 0;
    while (true)
    {
        MatchResult white_match = LinesMatcher::refineMatchWithWhitePoints(
            white_points,
            image_lines_map,
            center,
            counter_x,
            counter_y,
            counter_yaw);

        // 如果本次匹配结果没有改善，则退出循环
        if (white_match.max_sum <= last_white_sum)
        {
            break;
        }

        // 更新计数器和上一次的最大匹配值
        counter_x += white_match.best_dx;
        counter_y += white_match.best_dy;
        counter_yaw += white_match.best_angle;
        last_white_sum = white_match.max_sum;
    }

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

    // printf("counter_x: %.2f, counter_y: %.2f, counter_yaw: %.2f\n", counter_x, counter_y, counter_yaw);

    // 绘制白点
    for (const auto &point : white_points)
    {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if (rotated_x >= 0 && rotated_x < image_match_result.cols &&
            rotated_y >= 0 && rotated_y < image_match_result.rows)
        {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0, 255, 0), -1);
        }
    }

    // 绘制红点
    for (const auto &point : red_points)
    {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if (rotated_x >= 0 && rotated_x < image_match_result.cols &&
            rotated_y >= 0 && rotated_y < image_match_result.rows)
        {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0, 0, 255), -1);
        }
    }

    // 绘制蓝点
    for (const auto &point : blue_points)
    {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if (rotated_x >= 0 && rotated_x < image_match_result.cols &&
            rotated_y >= 0 && rotated_y < image_match_result.rows)
        {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(255, 0, 0), -1);
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
    info_area.setTo(cv::Scalar(255, 255, 255)); // 清除之前的文本

    std::string coordinates = cv::format("X: %.2f  Y: %.2f  Yaw: %.2f", counter_x, counter_y, counter_yaw);
    cv::putText(monitor_image, coordinates, cv::Point(200, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);

    // 绘制机器人图标
    // 计算field_image的中心点
    cv::Point2f bottom_center(field_image.cols / 2.0f, field_image.rows / 2.0f + 50);

    // 计算机器人在图像中的位置
    float robot_x = bottom_center.x + counter_x * 1.0;
    float robot_y = bottom_center.y + counter_y * 1.0;
    cv::Point robot_pos(robot_x, robot_y);

    // 绘制机器人主体（紫色填充的圆形，黑色轮廓）
    int robot_radius = 15;
    cv::circle(monitor_image, robot_pos, robot_radius, cv::Scalar(0, 0, 0), 2);          // 黑色轮廓
    cv::circle(monitor_image, robot_pos, robot_radius - 2, cv::Scalar(255, 0, 255), -1); // 紫色填充

    // 绘制朝向线段
    float direction_length = 20.0f;
    float direction_rad = counter_yaw * CV_PI / 180.0;
    cv::Point direction_end(
        robot_x + direction_length * cos(direction_rad),
        robot_y + direction_length * sin(direction_rad));
    cv::line(monitor_image, robot_pos, direction_end, cv::Scalar(0, 0, 0), 2);

    // 显示
    cv::imshow("result", image_show);
    // cv::imshow("scope", image_scope);
    cv::imshow("match_result", image_match_result);
    cv::imshow("定位", monitor_image);
    cv::waitKey(1);
}

// 添加重定位回调函数
void relocCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    counter_x = msg->pose.position.x;
    counter_y = msg->pose.position.y;

    // 从四元数转换为欧拉角(yaw)
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    counter_yaw = yaw * 180.0 / M_PI; // 转换为角度
    ROS_WARN("重新定位: ( %.2f,  %.2f) yaw: %.2f", counter_x, counter_y, counter_yaw);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "loc_sidelines");
    ros::NodeHandle nh("~");

    // int blue_h_max, blue_h_min, gray_s_threhold, white_v_threhold;
    nh.param<int>("blue_h_max", blue_h_max, 145);         // 290/2
    nh.param<int>("blue_h_min", blue_h_min, 65);          // 130/2
    nh.param<int>("red_h_max", red_h_max, 145);         // 290/2
    nh.param<int>("red_h_min", red_h_min, 65);          // 130/2
    nh.param<int>("gray_s_threhold", gray_s_threhold, 10);
    nh.param<int>("gray_v_threhold", gray_v_threhold, 10);
    nh.param<int>("white_v_min", white_v_min, 200);
    nh.param<int>("white_v_max", white_v_max, 200);

    // BLUE_H_MAX = static_cast<char>(blue_h_max);
    // BLUE_H_MIN = static_cast<char>(blue_h_min);
    // GRAY_S_THREHOLD = static_cast<char>(gray_s_threhold);
    // WHITE_V_THREHOLD = static_cast<char>(white_v_threhold);

    std::string table_file;
    nh.param<std::string>("table_file", table_file, "distances.txt");
    distanceLookup.init(table_file);

    std::string lines_file;
    nh.param<std::string>("lines_file", lines_file, "lines.jpg");
    image_lines_all = cv::imread(lines_file);

    std::string field_file;
    nh.param<std::string>("field_file", field_file, "field_bg.png");
    cv::Mat resultImage = cv::imread(field_file);
    if (resultImage.empty())
    {
        ROS_ERROR("读取场地背景图失败: %s", field_file.c_str());
        return -1;
    }

    // 初始化field_image为彩色图像，尺寸为resultImage的一半
    cv::resize(resultImage, field_image, cv::Size(resultImage.cols / 2, resultImage.rows / 2));

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
    if (image_lines_map.empty() || image_red_map.empty() || image_blue_map.empty())
    {
        ROS_ERROR("场线模板读取失败！");
        return -1;
    }

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);
    // 添加重定位话题订阅
    ros::Subscriber reloc_sub = nh.subscribe("/reloc_pose", 1, relocCallback);

    ros::spin();

    return 0;
}
