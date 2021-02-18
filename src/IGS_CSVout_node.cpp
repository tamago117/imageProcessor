#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <vector>
#include <string>
#include "CSVout.h"

const std::string filePath = "/home/user/catkin_ws/src/magImage/config/sensorData";
const double psize = 0.75;

std::vector<float> sensorValue;
void sensor_callback(const std_msgs::Float32MultiArray& sensorMessage)
{
    sensorValue = sensorMessage.data;
}

float movement,yaw;
void vel_callback(const std_msgs::Float32MultiArray& velMessage)
{
    movement = velMessage.data[0];
    yaw = velMessage.data[1];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "IGS_CSVout_node");
    ros::NodeHandle nh;
    ros::Subscriber sensorSub = nh.subscribe("sensorValue", 10, sensor_callback);
    ros::Subscriber velSub = nh.subscribe("magImage/state_vel", 10, vel_callback);
    CSVout csv(filePath);

    double preMovement = 0;
    ros::Rate loop_rate(200);
    while(ros::ok())
    {

        //csvColum.resize(32);
        double diff = movement - preMovement;
        double pixel = diff/psize;
        
        std::cout<<pixel<<std::endl;
        if(pixel<1000){
            for(int k=0; k<pixel; ++k){
                //csvColum.assign(sensorValue.begin(), sensorValue.end());
                csv.to_csv(sensorValue);
            }
        }
        preMovement = movement;

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
