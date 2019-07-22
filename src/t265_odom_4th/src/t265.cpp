#include "../include/t265.h"

#include <fstream>

#include <tf/transform_broadcaster.h>

MyT265Pipeline::MyT265Pipeline(ros::NodeHandle node_handle, 
                               ros::NodeHandle private_node_handle, 
                               rs2::device dev, 
                               rs2::pipeline ppl):
                               _nh(node_handle), _pnh(private_node_handle),
                               _wo_snr(dev.first<rs2::wheel_odometer>()),
                               _ppl(ppl),
                               _use_odom_input(false){
    
    bool use_odometry;
    _pnh.param("use_odometry", use_odometry, true);
    if (use_odometry) {
        setupOdomSub();
    }
    setupOdomPub();
}

MyT265Pipeline::~MyT265Pipeline() {

}

void MyT265Pipeline::setupOdomPub() {
    std::string topic_out;
    _pnh.param("topic_odom_out", topic_out, std::string(""));
    _pub = _nh.advertise<nav_msgs::Odometry>(topic_out, 10);
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ROS_INFO("publishing ...");
        rs2::frameset frames = _ppl.wait_for_frames();
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

        _pub.publish(odom_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MyT265Pipeline::setupOdomSub() {
    initOdomSetup();
    std::string topic_in;
    _pnh.param("topic_odom_in", topic_in, std::string(""));
    _sub = _nh.subscribe(topic_in, 10, &MyT265Pipeline::subCallBack, this);
}

void MyT265Pipeline::initOdomSetup() {
    std::string calib_odom_file;
    _pnh.param("calib_odom_file", calib_odom_file, std::string(""));
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

    if (!_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_INFO("calibaration file format error");
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    _use_odom_input = true;
    ROS_INFO_STREAM("initial odom input successfully");
}

void MyT265Pipeline::subCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("odomcallback is running");
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                        (float)(msg->twist.twist.linear.z),
                        -(float)(msg->twist.twist.linear.x)};
    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}