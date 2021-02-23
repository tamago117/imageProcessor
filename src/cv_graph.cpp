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

const int height = 320;
const int graphHeight = 300;
const int graphmargin = (height-graphHeight)/2;
const int expanRate = 15;

class cv_graph
{
ros::NodeHandle nh;
image_transport::ImageTransport it;
ros::Subscriber vecSub;
image_transport::Publisher graphImgPub;

private:
    cv::Mat image;
    std::vector<float> vecData_row;
    std::vector<float> vecData;
    template <class T> T clip(const T& n, float lower, float upper);
    template <class T> T normalize(const T& n, float xmin, float xmax ,float amin, float amax);
    float minValue(const std::vector<float>& value);
    float maxValue(const std::vector<float>& value);
    void vec_callback(const std_msgs::Float32MultiArray& vec_message);
public:
    cv_graph();
    ~cv_graph();
};

cv_graph::cv_graph():it(nh)
{
    graphImgPub = it.advertise("graphImage", 10);
    vecSub = nh.subscribe("sensorValue", 10, &cv_graph::vec_callback, this);
}

cv_graph::~cv_graph()
{
    //cv::destroyAllWindows();
}


template <class T> T cv_graph::clip(const T& n, float lower, float upper)
{
    T numbers = n;
    for(auto& number : numbers)
    {
        number = std::max(lower, std::min(number, upper));
    }
  return numbers;
}

template <class T> T cv_graph::normalize(const T& n, float xmin, float xmax ,float amin, float amax)
{
    T numbers = n;
    for(auto& number : numbers){
        number = (amax - amin) * (number - xmin) / (xmax - xmin) + amin;
    }
    return numbers;
}

float cv_graph::minValue(const std::vector<float>& value)
{
    float minV = 99999999;

    for(int i=0; i<value.size(); i++){
        if(minV>value[i]){
            minV = value[i];
        }
    }

    return minV;

}

float cv_graph::maxValue(const std::vector<float>& value)
{
    float maxV = -99999999;

    for(int i=0; i<value.size(); i++){
        if(maxV<value[i]){
            maxV = value[i];
        }
    }

    return maxV;

}

void cv_graph::vec_callback(const std_msgs::Float32MultiArray& vec_message)
{
    float maxVecValue = 0.1;
    float minVecValue = -0.1;

    vecData_row = vec_message.data;
    //->minV~maxV
    vecData = clip(vecData_row, minVecValue, maxVecValue);
    //->0~255
    vecData = normalize(vecData, minVecValue, maxVecValue, 0, graphHeight);

    image = cv::Mat(height, vecData_row.size()*(expanRate-1)+graphmargin*2, CV_8UC3, cv::Scalar(255, 255, 255));

    //centerLine
    cv::line(image, cv::Point(0, height/2), cv::Point(vecData_row.size()*(expanRate-1)+graphmargin*2, height/2), cv::Scalar(0,0,0), 3, 4);
    for(int i = 0;i<vecData_row.size()-1;i++){
        cv::line(image, cv::Point(i*expanRate+graphmargin, graphHeight-vecData[i]+graphmargin), cv::Point((i+1)*expanRate+graphmargin, graphHeight-vecData[i+1]+graphmargin), cv::Scalar(0,0,255), 2, 4);
    }

    cv::putText(image, std::to_string(maxValue(vecData_row)), cv::Point(25,15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);
    cv::putText(image, std::to_string(minValue(vecData_row)), cv::Point(25,height-15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);

    sensor_msgs::ImagePtr graphImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    graphImgPub.publish(graphImage);

    //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    //cv::imshow("image", image);
    //cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_graph");
    cv_graph graph;

    ros::spin();
    return 0;
}