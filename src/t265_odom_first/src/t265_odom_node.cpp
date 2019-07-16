#include "../include/t265_odom.h"

#include <iostream>

#include <ros/ros.h>



int main(int argc, char** argv) {
    
    std::string serial_no;

    ros::init(argc, argv, "rjp_t265_odom");
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh("~");
    _pnh.param("serial_no", serial_no, std::string(""));
    std::cout << "start node " << std::endl;
    std::cout << serial_no << std::endl;
    MyT265Camera t265(_nh, _pnh, serial_no);

    return 0;
}