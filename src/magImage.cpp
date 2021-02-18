#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "sensorImage.h"

const int height = 1000;
const int width = 32;
const double pixelSize = 0.75; //(mm/pix)

template <class T> T clip(const T& n, float lower, float upper)
{
    T numbers = n;
    for(auto& number : numbers)
    {
        number = std::max(lower, std::min(number, upper));
    }
  return numbers;
}

template <class T> T normalize(const T& n, float xmin, float xmax ,float amin=0, float amax=1)
{
    T numbers = n;
    for(auto& number : numbers){
        number = (amax - amin) * (number - xmin) / (xmax - xmin) + amin;
    }
    return numbers;
}

std::vector<float> sensorVal_row;
std::vector<uint8_t> sensorVal;
void sensorVal_callback(const std_msgs::Float32MultiArray& sensor_message)
{
    double maxValue = 0.5;
    double minValue = -0.5;

    sensorVal_row.resize(width);
    sensorVal_row = sensor_message.data;
    //->minV~maxV
    sensorVal_row = clip(sensorVal_row, minValue, maxValue);
    //->0~255
    sensorVal_row = normalize(sensorVal_row, minValue, maxValue, 0, 255);
    //->uint8
    sensorVal.assign(sensorVal_row.begin(), sensorVal_row.end());
}

float movement,yaw;
std::vector<float> stateVel;
void velArray_callback(const std_msgs::Float32MultiArray& velArray_message)
{
    stateVel.resize(2);
    stateVel = velArray_message.data;
    movement = velArray_message.data[0];
    yaw = velArray_message.data[1];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magImage");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber sensorSub = nh.subscribe("sensorValue", 10, sensorVal_callback);
    ros::Subscriber velArraySub = nh.subscribe("magImage/state_vel", 10, velArray_callback);
    image_transport::Publisher proImgPub = it.advertise("magImage", 10);
    image_transport::Publisher debImgPub = it.advertise("debugImage", 10);
    image_transport::Publisher corImgPub = it.advertise("correctionImage", 10);

    sensorImage senImg(height, width, pixelSize);

    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        cv::Mat image_dist = senImg.debugImageGeneration(sensorVal);
        //cv::Mat image_dist = senImg.imageGeneration_dist(sensorVal, movement);
        //cv::Mat image_dist = senImg.imageGeneration_cor(sensorVal);
        //sensor_msgs::ImagePtr pubProImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        sensor_msgs::ImagePtr pubImage_dist = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_dist).toImageMsg();

        //proImgPub.publish(pubProImage);
        debImgPub.publish(pubImage_dist);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}