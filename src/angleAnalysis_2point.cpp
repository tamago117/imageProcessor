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

const int minDist = 300;
const int param1 = 20;
const int param2 = 32;
const int maxRadius = 150;
const int minRadius = 60;

class angleAnalysis
{
ros::NodeHandle nh;
image_transport::ImageTransport it;
image_transport::Subscriber image_sub;
image_transport::Publisher proImgPub;
ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("result", 10);

private:
    cv::Mat image, image_g;
    double minX, minY, minR, maxX, maxY, maxR;
    double cenX, cenY;
    double x1, x2, y1, y2;
    double theta;
    double isDetect;
    void circleAnalysis(const std::vector<cv::Vec3f>& circles);
    int minCircle(const std::vector<float>& r);
    int maxCircle(const std::vector<float>& r);
public:
    angleAnalysis();
    ~angleAnalysis();
    void image_callback(const sensor_msgs::ImageConstPtr& image_message);
};

angleAnalysis::angleAnalysis():it(nh)
{
    image_sub = it.subscribe("image_raw", 10, &angleAnalysis::image_callback, this);
    proImgPub = it.advertise("processedImage", 10);
}

angleAnalysis::~angleAnalysis()
{
    //cv::destroyAllWindows();
}

int angleAnalysis::minCircle(const std::vector<float>& r)
{
    int minRadius = 99999999;
    int minC;

    for(int i=0; i<r.size(); i++){
        if(minRadius>r[i]){
            minRadius = r[i];
            minC = i;
        }
    }

    return minC;

}

int angleAnalysis::maxCircle(const std::vector<float>& r)
{
    int maxRadius = 0;
    int maxC;

    for(int i=0; i<r.size(); i++){
        if(maxRadius<r[i]){
            maxRadius = r[i];
            maxC = i;
        }
    }

    return maxC;

}

void angleAnalysis::circleAnalysis(const std::vector<cv::Vec3f>& circles)
{

    int i = 0;
    std::vector<float> x(circles.size(), 0);
    std::vector<float> y(circles.size(), 0);
    std::vector<float> r(circles.size(), 0);
    for(const auto& circle : circles){
        //std::cout<<1<<std::endl;
        x[i] = circle[0];
        y[i] = circle[1];
        r[i] = circle[2];
        i++;
        //cv::circle(image_g, cv::Point(circle[0], circle[1]), circle[2], cv::Scalar(0, 0, 255), 2);
    }

    minX = x[minCircle(r)];
    minY = y[minCircle(r)];
    minR = r[minCircle(r)];
    maxX = x[maxCircle(r)];
    maxY = y[maxCircle(r)];
    maxR = r[maxCircle(r)];

    if(minX>maxX){
        cenX = (minX - maxX)/2 + maxX;
        x1 = minX - cenX;
        x2 = cenX - maxX;
    }else{
        cenX = (maxX -minX)/2 + minX;
        x1 = cenX - minX;
        x2 = maxX - cenX;
    }if(minY>maxY){
        cenY = (minY - maxY)/2 + maxY;
        y1 = minY - cenY;
        y2 = cenY - maxY;
    }else{
        cenY = (maxY - minY)/2 + minY;
        y1 = cenY - minY;
        y2 = maxY - cenY;
    }

    if(minX>maxX && minY>maxY){
        theta = -atan2(y1, x1)*180/M_PI + 90;
    }else if(minX>maxX){
        theta = atan2(y2, x1)*180/M_PI + 90;
    }else if(minY>maxY){
        theta = atan2(y1, x2)*180/M_PI + 90;
    }else{
        theta = -atan2(y2, x2)*180/M_PI + 90;
    }

    std::cout<<theta<<std::endl;
}

void angleAnalysis::image_callback(const sensor_msgs::ImageConstPtr& image_message)
{
    try {
        //image = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8)->image;
        image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::cvtColor(image, image_g, CV_BGR2GRAY);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(image_g, circles, CV_HOUGH_GRADIENT, 1, minDist, param1, param2, minRadius, maxRadius);
    if(circles.size() <= 1){
        isDetect = 0;
        sensor_msgs::ImagePtr pubProImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_g).toImageMsg();
        proImgPub.publish(pubProImage);
        return;
    }
    isDetect = 1;
    circleAnalysis(circles);

    cv::line(image_g, cv::Point(minX, minY), cv::Point(maxX, maxY), cv::Scalar(0,0,255), 3, 4);
    cv::circle(image_g, cv::Point(minX, minY), minR, cv::Scalar(0,0,200), 3, 4);
    cv::circle(image_g, cv::Point(maxX, maxY), maxR, cv::Scalar(0,0,200), 3, 4);
    cv::putText(image_g, std::to_string(theta), cv::Point(25,75), cv::FONT_HERSHEY_SIMPLEX, 2.5, cv::Scalar(0,0,0), 3);

    std_msgs::Float32MultiArray result;
    result.data.resize(2);
    result.data[0] = theta+90;
    result.data[1] = isDetect;

    sensor_msgs::ImagePtr pubProImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_g).toImageMsg();
    proImgPub.publish(pubProImage);
    pub.publish(result);

    //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    //cv::imshow("image", image);
    //cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "angleAnalysis");
    angleAnalysis angleAn;

    ros::spin();
    return 0;
}