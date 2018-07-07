//
// Created by leo on 18-6-5.
//

#include "kaka_base_serial.h"


KakaBaseSerial::KakaBaseSerial()
{
    param_node.param<string>("serial_port_name",serial_port_name,"/dev/ttyUSB0");
    param_node.param<int>("serial_baud_rate",serial_baud_rate,115200);

    serial_sub = node.subscribe("/cmd_vel",1000,&KakaBaseSerial::CmdVelCallBack,this);

    serial = new serial::Serial;
    encode_abs.resize(4);
    coefficient_t = 2700/(0.05*2*M_PI);
    coefficient_k = 0.4775;
    matrix_f.resize(3,4);
    matrix_f << 0.25, 0.25,  0.25, 0.25,
               -0.25, 0.25, -0.25, 0.25,
               -0.25/coefficient_k, -0.25/coefficient_k, 0.25/coefficient_k, 0.25/coefficient_k;
}

void KakaBaseSerial::Run()
{
    ROS_INFO("[KAKA_BASE_SERIAL] Running");
    OpenSerialPort();//如果一直打不开可能是串口权限的问题
    ros::Rate Rate(1000);
    while (ros::ok())
    {
        if(IsOpened())
        {
            ReadSerialData();
        }
        else
        {
            ROS_INFO("[KAKA_BASE_SERIAL] Open serial port failed, try again");
            OpenSerialPort();
        }
        Rate.sleep();
        ros::spinOnce();
    }
}

//打开串口
bool KakaBaseSerial::OpenSerialPort()
{
    try
    {
        serial->setPort(serial_port_name);
        serial->setBaudrate(serial_baud_rate);
        serial::Timeout Time = serial::Timeout::simpleTimeout(1000);
        serial->setTimeout(Time);
        serial->open();
        ROS_INFO("[KAKA_BASE_SERIAL] Open serial port success!");
        return true;
    }
    catch (serial::IOException &Err)
    {
        ROS_INFO("[KAKA_BASE_SERIAL] Unable to open serial port");
        return false;
    }
}

//判断串口是否打开
bool KakaBaseSerial::IsOpened()
{
    if(serial->isOpen())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void KakaBaseSerial::CmdVelCallBack(const geometry_msgs::Twist::ConstPtr &Msg)
{
    unsigned short px,py,pz;
    char t = 0;
    px = (unsigned short)abs(Msg->linear.x*coefficient_t);//按照串口协议解析发送
    py = (unsigned short)abs(Msg->linear.y*coefficient_t);
    pz = (unsigned short)abs(Msg->angular.z*coefficient_t);
    if(Msg->linear.x<0)
    {
        t = t|(1<<2);
    }
    if(Msg->linear.y<0)
    {
        t = t|(1<<1);
    }
    if(Msg->angular.z<0)
    {
        t = t|(1<<0);
    }

    vector<unsigned char> send_cmd;
    send_cmd.push_back(0xff);
    send_cmd.push_back(0xfe);
    send_cmd.push_back(0x01);
    send_cmd.push_back((px>>8)&0xff);
    send_cmd.push_back((px)&0xff);
    send_cmd.push_back((py>>8)&0xff);
    send_cmd.push_back((py)&0xff);
    send_cmd.push_back((pz>>8)&0xff);
    send_cmd.push_back((pz)&0xff);
    send_cmd.push_back(t);

//    for(vector<unsigned char>::iterator it = send_cmd.begin(); it != send_cmd.end(); it++)
//    {
//        cout<<hex<<(int)*it<<endl;
//    }

    if(IsOpened())
    {
        serial->write(send_cmd);
    }
}

void KakaBaseSerial::ReadSerialData()
{
    vector<unsigned char> read_data;

    current_time = clock();
    float duration = ((double)(current_time - previous_time))/CLOCKS_PER_SEC;//开机不能动
    cout<<duration<<endl<<endl;
    previous_time = current_time;

    if(serial->available())
    {
        serial->read(read_data, 12);
        //下面对接收到的数据进行解析
        if(read_data.at(0) == 0xff && read_data.at(1) == 0xfe)
        {
            vector<int> encode_diff;
            unsigned  short gyro_z;
            encode_diff.push_back((int)read_data.at(4)*(1-read_data.at(5)));
            encode_diff.push_back((int)read_data.at(2)*(1-read_data.at(3)));
            encode_diff.push_back((int)read_data.at(8)*(1-read_data.at(9)));
            encode_diff.push_back((int)read_data.at(6)*(1-read_data.at(7)));
            gyro_z = (unsigned short)read_data.at(10)<<8 | read_data.at(11);
            for(int i=0; i<encode_diff.size(); i++)
            {
                encode_abs.at(i) = encode_abs.at(i) + encode_diff.at(i);
                cout<<encode_abs.at(i)<<endl;
                distance[i] = encode_abs.at(i)/coefficient_t;
            }
            cout<<endl;
            //cout<<distance<<endl<<endl;
            Vector3d odometry;
            odometry = matrix_f * distance;
            //cout<<odometry<<endl<<endl;
        }

    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"kaka_base_serial");
    KakaBaseSerial KakaBaseSerial;
    KakaBaseSerial.Run();
    return 0;
}
