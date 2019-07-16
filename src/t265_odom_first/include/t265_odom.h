#include <iostream>
#include <fstream>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <librealsense2/rs.hpp>

class MyT265Camera {
public:
    MyT265Camera(ros::NodeHandle node_handle,                            
                 ros::NodeHandle private_node_handle,
                 std::string serial_no);
    ~MyT265Camera();
    

private:
    rs2::device getDevice(std::string serial_no);
    void initialOdomInput();
    
    void setupOdomSub();
    void subCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void setupOdomPub();
private:
    ros::NodeHandle& _nh, _pnh;
    ros::Subscriber _input_odom_sub;
    ros::Publisher _output_odom_pub;
    ros::Time _current_time;
    tf::TransformBroadcaster _tf_br;

    rs2::device _rs2_dev;
    rs2::wheel_odometer _rs2_wo_snr;    
    bool _use_input_odom;

    rs2::pipeline _rs2_pipe;
    rs2::config _rs2_cfg;
    rs2::frameset _rs2_frames;
    rs2::frame _rs2_frame;
    rs2_pose _rs2_pose_data;
};
