#include "../include/t265_odom.h"

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <librealsense2/rs.hpp>

rs2::device MyT265Camera::getDevice(std::string serial_no) {
    std::cout << "start get device function" << std::endl;
    rs2::context context;
    rs2::device_list device_list = context.query_devices();
    // std::cout << device_list.size() << std::endl;
    for (int i = 0; i < device_list.size(); i++) {
        // std::cout << "*****************************" << std::endl;
        std::string tem_serial_no = device_list[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
        // std::cout << "tem_serial_no" << tem_serial_no << std::endl;
        // std::cout << tem_serial_no.find(serial_no) << std::endl;
        if (tem_serial_no == serial_no) {
            // std::cout << "Find the device of serial number " << serial_no << std::endl;
            return device_list[i];
        }
    }
}

MyT265Camera::MyT265Camera(ros::NodeHandle node_handle, 
                           ros::NodeHandle private_node_handle, 
                           std::string serial_no):
                           _nh(node_handle), _pnh(private_node_handle),
                           _use_input_odom(false),
                           _rs2_dev(getDevice(serial_no)),
                           _rs2_wo_snr(_rs2_dev.first<rs2::wheel_odometer>()) {
    initialOdomInput();
    setupOdomSub();
    setupOdomPub();

}

void MyT265Camera::initialOdomInput() {
    std::string calib_odom_file;
    _pnh.param("calib_odom_file", calib_odom_file, std::string(""));
    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        return;
    }
    std::ifstream calibrationFile(calib_odom_file);
    if (not calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_rs2_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    _use_input_odom = true;
}

void MyT265Camera::setupOdomSub() {
    if (not _use_input_odom) return;

    std::string topic_odom_in;
    _pnh.param("topic_odom_in", topic_odom_in, std::string(""));
    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);
    _input_odom_sub = _nh.subscribe<nav_msgs::Odometry>(topic_odom_in, 1, &MyT265Camera::subCallback, this);
}

void MyT265Camera::subCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("sub callback is running");
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};
    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _rs2_wo_snr.send_wheel_odometry(0, 0, velocity);
}

void MyT265Camera::setupOdomPub() {
    std::string topic_odom_out;
    _pnh.param("topic_odom_out", topic_odom_out, std::string(""));
    _output_odom_pub = _nh.advertise<nav_msgs::Odometry>(topic_odom_out, 10);

    int loop_frequence;
    _pnh.param("pub_frequence", loop_frequence, 50);
    ros::Rate loop_rate(loop_frequence);

    _rs2_cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    _rs2_pipe.start(_rs2_cfg);
    
    while(ros::ok()) {
        ROS_INFO("pub is running");
        _rs2_frames = _rs2_pipe.wait_for_frames();
        _rs2_frame = _rs2_frames.first_or_default(RS2_STREAM_POSE);
        _rs2_pose_data = _rs2_frame.as<rs2::pose_frame>().get_pose_data();

        // set up tf transform
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = -_rs2_pose_data.translation.z;
        pose_msg.pose.position.y = -_rs2_pose_data.translation.x;
        pose_msg.pose.position.z = _rs2_pose_data.translation.y;
        pose_msg.pose.orientation.x = -_rs2_pose_data.rotation.z;
        pose_msg.pose.orientation.y = -_rs2_pose_data.rotation.x;
        pose_msg.pose.orientation.z = _rs2_pose_data.rotation.y;
        pose_msg.pose.orientation.w = _rs2_pose_data.rotation.w;

        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "t265_odom";
        msg.child_frame_id = "base_link";
        msg.transform.translation.x = pose_msg.pose.position.x;
        msg.transform.translation.y = pose_msg.pose.position.y;
        msg.transform.translation.z = pose_msg.pose.position.z;
        msg.transform.rotation.x = pose_msg.pose.orientation.x;
        msg.transform.rotation.y = pose_msg.pose.orientation.y;
        msg.transform.rotation.z = pose_msg.pose.orientation.z;
        msg.transform.rotation.w = pose_msg.pose.orientation.w;
        _tf_br.sendTransform(msg);

        // pre for the odom be send
        geometry_msgs::Vector3Stamped v_msg;
        v_msg.vector.x = -_rs2_pose_data.velocity.z;
        v_msg.vector.y = -_rs2_pose_data.velocity.x;
        v_msg.vector.z = _rs2_pose_data.velocity.y;
        tf::Vector3 tfv;
        tf::vector3MsgToTF(v_msg.vector,tfv);
        tf::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,v_msg.vector);

        geometry_msgs::Vector3Stamped om_msg;
        om_msg.vector.x = -_rs2_pose_data.angular_velocity.z;
        om_msg.vector.y = -_rs2_pose_data.angular_velocity.x;
        om_msg.vector.z = _rs2_pose_data.angular_velocity.y;
        tf::vector3MsgToTF(om_msg.vector,tfv);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,om_msg.vector);

        // odom
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "t265_odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        
        _output_odom_pub.publish(odom_msg);
        
        loop_rate.sleep();
    }
}

MyT265Camera::~MyT265Camera() {

}