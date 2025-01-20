#ifndef LINES_MAP_H
#define LINES_MAP_H

#include <opencv2/opencv.hpp>
#include <string>

class LinesMap {
public:
    LinesMap();

    void loadImage(const std::string& imagePath);
    cv::Mat getLinesMap() const;
    cv::Mat getRawImage() const;

private:
    cv::Mat createGradientMask(int size);
    void generateGradientImage();

    cv::Mat lines_image;
    cv::Mat grayImage;
    cv::Mat gradientImage;
};

#endif // LINES_MAP_H 