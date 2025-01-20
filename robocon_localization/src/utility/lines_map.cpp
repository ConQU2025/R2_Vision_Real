#include "lines_map.h"
#include <ros/ros.h>

LinesMap::LinesMap() {
}

void LinesMap::loadImage(const std::string& imagePath) {
    lines_image = cv::imread(imagePath);
    if (lines_image.empty()) {
        ROS_ERROR("打不开图片文件: %s", imagePath.c_str());
        return;
    }
    cv::cvtColor(lines_image, grayImage, cv::COLOR_BGR2GRAY);
    generateGradientImage();
}

cv::Mat LinesMap::createGradientMask(int size) {
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
}

void LinesMap::generateGradientImage() {
    int maskSize = 200;
    cv::Mat mask = createGradientMask(maskSize);
    gradientImage = grayImage.clone();

    for (int y = 0; y < grayImage.rows; y++) {
        for (int x = 0; x < grayImage.cols; x++) {
            if (grayImage.at<uchar>(y, x) == 255) {
                for (int dy = -maskSize/2; dy < maskSize/2; dy++) {
                    for (int dx = -maskSize/2; dx < maskSize/2; dx++) {
                        int newY = y + dy;
                        int newX = x + dx;
                        
                        if (newY >= 0 && newY < gradientImage.rows && 
                            newX >= 0 && newX < gradientImage.cols) {
                            int maskY = dy + maskSize/2;
                            int maskX = dx + maskSize/2;
                            uchar maskValue = mask.at<uchar>(maskY, maskX);
                            
                            if (maskValue > gradientImage.at<uchar>(newY, newX)) {
                                gradientImage.at<uchar>(newY, newX) = maskValue;
                            }
                        }
                    }
                }
            }
        }
    }
}

cv::Mat LinesMap::getLinesMap() const {
    return gradientImage;
}

cv::Mat LinesMap::getRawImage() const {
    return lines_image.clone();
}
