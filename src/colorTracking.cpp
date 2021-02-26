#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>
#include <math.h>

const uint8_t minH = 5;
const uint8_t minS = 100;
const uint8_t minV = 0;
const uint8_t maxH = 20;
const uint8_t maxS = 255;
const uint8_t maxV = 255;

class colorTracking
{
ros::NodeHandle nh;
image_transport::ImageTransport it;
image_transport::Subscriber image_sub;
image_transport::Publisher proImgPub;
ros::Publisher posPub = nh.advertise<std_msgs::Float32MultiArray>("result", 10);

private:
    int centerX, centerY;
    cv::Mat image, image_g, mask;
    std::vector<float> vecData_row;
    std::vector<float> vecData;
    template <class T> T clip(const T& n, float lower, float upper);
    template <class T> T normalize(const T& n, float xmin, float xmax ,float amin, float amax);
    float minValue(const std::vector<float>& value);
    float maxValue(const std::vector<float>& value);
    cv::Mat colorBinarization(const cv::Mat& frame);
    void maxContourAnalysis(const cv::Mat& mask);
    void image_callback(const sensor_msgs::ImageConstPtr& image_message);
public:
    colorTracking();
    ~colorTracking();
};

colorTracking::colorTracking():it(nh)
{
    proImgPub = it.advertise("processedImage", 10);
    image_sub = it.subscribe("image_raw", 10, &colorTracking::image_callback, this);
}

colorTracking::~colorTracking()
{
    //cv::destroyAllWindows();
}

float colorTracking::minValue(const std::vector<float>& value)
{
    float minV = 99999999;

    for(int i=0; i<value.size(); i++){
        if(minV>value[i]){
            minV = value[i];
        }
    }

    return minV;

}

float colorTracking::maxValue(const std::vector<float>& value)
{
    float maxV = -99999999;

    for(int i=0; i<value.size(); i++){
        if(maxV<value[i]){
            maxV = value[i];
        }
    }

    return maxV;

}

cv::Mat colorTracking::colorBinarization(const cv::Mat& frame)
{
    cv::Mat hsv_image, hsv_min, hsv_max;
    cv::cvtColor(frame, hsv_image, CV_BGR2HSV, 3);
    //binarization
	cv::inRange(hsv_image, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);
    //dilate
    cv::Mat kernel(5,5,CV_8U, cv::Scalar(1));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 10);
    return mask;
}

void colorTracking::maxContourAnalysis(const cv::Mat& mask)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(const auto& contour: contours){
        if (contour.size() < 150) continue;
        //輪郭凸包
        std::vector<cv::Point> approx;
        cv::convexHull(contour, approx);
        cv::Mat pointsf;
        cv::Mat(contour).convertTo(pointsf, CV_32F);
        // 楕円フィッティング
        cv::RotatedRect box = cv::fitEllipse(pointsf);
        // 楕円の描画
        cv::ellipse(image, box, cv::Scalar(0x00, 0xFF), 2, cv::LINE_AA);
        centerX = box.center.x - mask.cols;
        centerY = box.center.y - mask.rows;
    }
}

void colorTracking::image_callback(const sensor_msgs::ImageConstPtr& image_message)
{
    try {
        image = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(image, image_g, CV_BGR2GRAY);
    mask = colorBinarization(image_g);
    maxContourAnalysis(mask);

    //cv::putText(image, std::to_string(maxValue(vecData_row)), cv::Point(25,15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);
    //cv::putText(image, std::to_string(minValue(vecData_row)), cv::Point(25,height-15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);

    sensor_msgs::ImagePtr proImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    proImgPub.publish(proImage);

    //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    //cv::imshow("image", image);
    //cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "colorTracking");
    colorTracking ct;

    ros::spin();
    return 0;
}