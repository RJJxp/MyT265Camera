#include "../include/t265.h"

#include <iostream>
#include <fstream>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <librealsense2/rs.hpp>



class wo_sender {
public: 
    wo_sender(rs2::device dev, ros::NodeHandle& node_handle): 
    _wo(dev.first<rs2::wheel_odometer>()),
    _nh(node_handle){

    }
    void callBack(const nav_msgs::Odometry::ConstPtr& msg) {
        ROS_INFO("odomcallback is running");
        ROS_DEBUG("Got in_odom message");
        rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};
        ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
        _wo.send_wheel_odometry(0, 0, velocity);
    }

    void run() {
        _sub = _nh.subscribe("/raccoon/chassis/odometry", 5, &wo_sender::callBack, this);
    }

    
public:
    ros::Subscriber _sub;
    rs2::wheel_odometer _wo;
    ros::NodeHandle& _nh;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "t265_3rd_node");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    rs2::pipeline ppl;
    rs2::pipeline_profile ppl_pf;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    ppl.start(cfg);
    ppl_pf = ppl.get_active_profile();
    rs2::device dev = ppl_pf.get_device();
    wo_sender wo_snr(dev, node_handle);
    // rs2::wheel_odometer wo_snr(dev.first<rs2::wheel_odometer>());

    // start of the subscriber
    std::string calib_odom_file;
    private_node_handle.param("calib_odom_file", calib_odom_file, std::string(""));
    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        throw std::runtime_error("111111111111" );
    }
    std::ifstream calibrationFile(calib_odom_file);
    if (not calibrationFile)
    {
        ROS_INFO("not calibrationfile");
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!wo_snr._wo.load_wheel_odometery_config(wo_calib))
    {
        ROS_INFO("calibaration file format error");
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    bool _use_odom_input = true;
    ROS_INFO_STREAM("initial odom input successfully");
    
    wo_snr.run();
    // end of subscriber

    ros::Publisher pub = node_handle.advertise<nav_msgs::Odometry>("t265/rjp/test", 5);
    ros::Rate loop_rate(50);

    while (ros::ok()) {
        ROS_INFO("publishing ...");
        rs2::frameset frames = ppl.wait_for_frames();
        rs2::frame frame = frames.first_or_default(RS2_STREAM_POSE);
        rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
        
        // publish
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = -pose.translation.z;
        pose_msg.pose.position.y = -pose.translation.x;
        pose_msg.pose.position.z = pose.translation.y;
        pose_msg.pose.orientation.x = -pose.rotation.z;
        pose_msg.pose.orientation.y = -pose.rotation.x;
        pose_msg.pose.orientation.z = pose.rotation.y;
        pose_msg.pose.orientation.w = pose.rotation.w;

        tf::TransformBroadcaster tf_br;
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "t265_frame";
        msg.child_frame_id = "base_link";
        msg.transform.translation.x = pose_msg.pose.position.x;
        msg.transform.translation.y = pose_msg.pose.position.y;
        msg.transform.translation.z = pose_msg.pose.position.z;
        msg.transform.rotation.x = pose_msg.pose.orientation.x;
        msg.transform.rotation.y = pose_msg.pose.orientation.y;
        msg.transform.rotation.z = pose_msg.pose.orientation.z;
        msg.transform.rotation.w = pose_msg.pose.orientation.w;
        tf_br.sendTransform(msg);

        geometry_msgs::Vector3Stamped v_msg;
        v_msg.vector.x = -pose.velocity.z;
        v_msg.vector.y = -pose.velocity.x;
        v_msg.vector.z = pose.velocity.y;
        tf::Vector3 tfv;
        tf::vector3MsgToTF(v_msg.vector,tfv);
        tf::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,v_msg.vector);

        geometry_msgs::Vector3Stamped om_msg;
        om_msg.vector.x = -pose.angular_velocity.z;
        om_msg.vector.y = -pose.angular_velocity.x;
        om_msg.vector.z = pose.angular_velocity.y;
        tf::vector3MsgToTF(om_msg.vector,tfv);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,om_msg.vector);
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "t265_frame";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;

        pub.publish(odom_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

