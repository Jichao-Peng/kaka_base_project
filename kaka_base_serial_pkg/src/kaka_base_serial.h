//
// Created by leo on 18-6-5.
//

#ifndef PROJECT_KAKA_BASE_SERIAL_H
#define PROJECT_KAKA_BASE_SERIAL_H

#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace Eigen;

class KakaBaseSerial
{
public:
    KakaBaseSerial();
    void Run();

private:
    int serial_baud_rate;
    string serial_port_name;
    ros::NodeHandle node;
    ros::NodeHandle param_node;
    ros::Subscriber serial_sub;
    ros::Publisher serial_pub;

    serial::Serial *serial;
    vector<int> encode_abs;
    Vector4d distance;

    double coefficient_t;
    double coefficient_k;
    MatrixXd matrix_f;

    clock_t current_time,previous_time;

    bool OpenSerialPort();
    bool IsOpened();
    void ReadSerialData();
    void CmdVelCallBack(const geometry_msgs::Twist::ConstPtr &Msg);
};


#endif //PROJECT_KAKA_BASE_SERIAL_H
