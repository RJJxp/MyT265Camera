#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <librealsense2/rs.hpp>

class MyT265Pipeline {

public:
    MyT265Pipeline(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle, rs2::device dev, rs2::pipeline ppl);
    ~MyT265Pipeline();
    
    void setupOdomSub();
    void setupOdomPub();

private:
    void subCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    void initOdomSetup();
private:
    ros::NodeHandle& _nh;
    ros::NodeHandle& _pnh;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    bool _use_odom_input;

    rs2::wheel_odometer _wo_snr;
    rs2::pipeline& _ppl;

};