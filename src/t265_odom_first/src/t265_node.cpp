#include <iostream>

#include <ros/ros.h>

#include "../include/t265.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rjp_t265_node");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    
    std::string serial_no;
    private_node_handle.param("serial_no", serial_no, std::string(""));
    ROS_INFO_STREAM("serial no is " << serial_no);

    MyT265Camera t265(node_handle, private_node_handle, serial_no);

    
    return 0;
}