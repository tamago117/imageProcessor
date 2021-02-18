#ifndef SENSOR_IMAGE
#define SENSOR_IMAGE

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>

#include "CppCubicSpline.h"

class sensorImage
{
private:
    std::uint32_t h, w;
    float psize;
    double pixel;
    const uint8_t expanRate = 20;
    const uint32_t resolution = 30;
    std::uint32_t img_corWidth;
    double preMovement = 0;
    cv::Mat img, debug_img, img_cor, img_dist, senArrayImg;
    CppCubicSpline CubicSpline;
public:
    sensorImage(std::uint32_t height, std::uint32_t width, float pixelSize);
    inline cv::Mat imageUpdate(const cv::Mat& image, const cv::Mat& sensorArrayImg);
    cv::Mat imageGeneration_row(const std::vector<uint8_t>& sensorValue);
    cv::Mat debugImageGeneration(const std::vector<uint8_t>& sensorValue);
    cv::Mat imageGeneration_cor(const std::vector<uint8_t>& sensorValue);
    cv::Mat imageGeneration_dist(const std::vector<uint8_t>& sensorValue, float movement);
};

sensorImage::sensorImage(std::uint32_t height, std::uint32_t width, float pixelSize):CubicSpline()
{
    h = height;
    w = width;
    psize = pixelSize;
    img_corWidth = width + (width - 1) * (resolution - 1);

    img = cv::Mat(h, w, CV_8U, cv::Scalar(127));
    debug_img = cv::Mat(h, w*expanRate, CV_8U, cv::Scalar(127));
    img_cor = cv::Mat(h, img_corWidth, CV_8U, cv::Scalar(127));
    //img_dist = cv::Mat(h, w*expanRate, CV_8U, cv::Scalar(127));
    img_dist = cv::Mat(h, img_corWidth, CV_8U, cv::Scalar(127));

    CppCubicSpline CubicSpline();
}

//image::h X w image, sensorArrayImage::only the 1 X w sensor colum
inline cv::Mat sensorImage::imageUpdate(const cv::Mat& image_message, const cv::Mat& sensorArrayImage_message)
{
    cv::Mat image = image_message;
    cv::Mat sensorArrayImage = sensorArrayImage_message;

    image.pop_back();
    sensorArrayImage.push_back(image);
    sensorArrayImage.copyTo(image);

    return image;
}

//32×height生画像
cv::Mat sensorImage::imageGeneration_row(const std::vector<uint8_t>& sensorValue)
{
    senArrayImg = cv::Mat(1, w, CV_8U, cv::Scalar(127));

    uint8_t *src = senArrayImg.ptr<uint8_t>(0);
    for(int i=0; i<sensorValue.size(); ++i){
        //senArrayImg.at<uchar>(0,i) = sensorValue[i];
        src[i] = sensorValue[i];
    }
    img = imageUpdate(img, senArrayImg);

    return img;
}

//拡張画像
cv::Mat sensorImage::debugImageGeneration(const std::vector<uint8_t>& sensorValue)
{
    senArrayImg = cv::Mat(1, w*expanRate, CV_8U, cv::Scalar(127));

    uint8_t *src = senArrayImg.ptr<uint8_t>(0);
    for(int i=0; i<sensorValue.size()*expanRate; ++i){
        src[i] = sensorValue[i/expanRate];
    }
    debug_img = imageUpdate(debug_img, senArrayImg);

    return debug_img;
}

//補正処理付き画像
cv::Mat sensorImage::imageGeneration_cor(const std::vector<uint8_t>& sensorValue)
{
    senArrayImg = cv::Mat(1, img_corWidth, CV_8U, cv::Scalar(127));
    CubicSpline.updateParameter(sensorValue);
    for(int i=0; i<img_corWidth; ++i){
        senArrayImg.at<uchar>(0,i) = CubicSpline.Calc((double)i/resolution);
        //std::cout<<i/30<<std::endl;
    }
    img_cor = imageUpdate(img_cor, senArrayImg);
    return img_cor;
}

//移動距離対応
cv::Mat sensorImage::imageGeneration_dist(const std::vector<uint8_t>& sensorValue, float movement)
{
    //拡張画像版
    /*senArrayImg = cv::Mat(1, w*expanRate, CV_8U, cv::Scalar(127));
    double diff = movement - preMovement;
    int pixel = diff/psize;
    uint8_t *src = senArrayImg.ptr<uint8_t>(0);

    std::cout<<pixel<<std::endl;
    if(pixel<2500){
        for(int k=0; k<pixel; ++k){
            #pragma omp parallel for
            for(int i=0; i<sensorValue.size()*expanRate; ++i){
                //senArrayImg.at<uchar>(0, i) = sensorValue[i/expanRate];
                src[i] = sensorValue[i/expanRate];
            }
            img_dist = imageUpdate(img_dist, senArrayImg);
        }
    }

    preMovement = movement;

    return img_dist;*/


//線形処理Ver
    senArrayImg = cv::Mat(1, img_corWidth, CV_8U, cv::Scalar(127));
    double diff = movement - preMovement;
    double pixel = diff/psize;
    uint8_t *src = senArrayImg.ptr<uint8_t>(0);

    CubicSpline.updateParameter(sensorValue);
    //std::cout<<pixel<<std::endl;
    if(pixel<1000){
        for(int k=0; k<pixel; ++k){
            #pragma omp parallel for
            for(int i=0; i<img_corWidth; ++i){
                //senArrayImg.at<uchar>(0,i) = CubicSpline.Calc(i/40);
                src[i] = CubicSpline.Calc((double)i/resolution);
            }
            img_dist = imageUpdate(img_dist, senArrayImg);
        }
    }

    preMovement = movement;

    return img_dist;
}


#endif