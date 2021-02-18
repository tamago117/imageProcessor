#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

class CSVout
{
private:
    //example : "/home/user/catkin_ws/src/magImage/config/sensorData"
    std::string filePath;
    std::string nowDay();
    std::ofstream ofs;
public:
    CSVout(std::string filePath);
    template<typename T> void to_csv(const T& data);
};

std::string CSVout::nowDay()
{
    time_t t = time(nullptr);
    //change format
    const tm* lt = localtime(&t);

    //sに独自フォーマットになるように連結していく
    std::stringstream s;
    s<<"20";
    s<<lt->tm_year-100; //100を引くことで20xxのxxの部分になる
    s<<"-";
    s<<lt->tm_mon+1; //月を0からカウントしているため
    s<<"-";
    s<<lt->tm_mday;
    s<<"-";
    s<<lt->tm_hour;
    s<<"-";
    s<<lt->tm_min;
    s<<"-";
    s<<lt->tm_sec;
    std::string result = s.str();

    return result;
}

CSVout::CSVout(std::string filePath):ofs((filePath + nowDay() + ".csv").c_str())
{
    //filePath = filePath + nowDay() + ".csv";
    //std::ofstream ofs(filePath.c_str());
}

template<typename T> void CSVout::to_csv(const T& data)
{
    for(const auto d : data){
        //std::cout<<d<<std::endl;
        ofs << d << ",";
    }
    ofs << std::endl;

}