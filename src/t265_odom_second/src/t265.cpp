#include "../include/t265.h"

#include <fstream>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

MyT265Camera::MyT265Camera(ros::NodeHandle& node_hanle,
                           ros::NodeHandle& private_node_handle,
                           std::string serial_no):
                           _nh(node_hanle), 
                           _pnh(private_node_handle),
                           _serial_no(serial_no),
                           _use_odom_input(false),
                           _device(getDevice()),
                           _wo_snr(_device.first<rs2::wheel_odometer>()){
    setupDevice();
    setupOdomPub();

    bool use_odometry;
    _pnh.param("use_odometry", use_odometry, true);
    if (use_odometry) {
        initialOdomInput();
        setupOdomSub();
    }
}

rs2::device MyT265Camera::getDevice() {
    ROS_INFO("start get device function");
    rs2::context context;
    rs2::device_list device_list = context.query_devices();
    ROS_INFO_STREAM("device list size is " << device_list.size());
    for (int i = 0; i < device_list.size(); i++) {
        std::string tem_serial_no = device_list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        ROS_INFO_STREAM("tem serial no is " << tem_serial_no);
        if (tem_serial_no  == _serial_no) {
            ROS_INFO("get the device successfully");
            return device_list[i];
        }
    }
}

void MyT265Camera::setupDevice() {
    std::vector<rs2::sensor> sensors = _device.query_sensors();
    ROS_INFO_STREAM("there are " << sensors.size() << " sensors");
    for (rs2::sensor& sensor:sensors) {
        std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        if ("Tracking Module" == module_name) {
            ROS_INFO_STREAM("find the sensor with " << module_name);
            _sensor = sensor;
            break;
        }
    }
    // for (int i = 0; i < sensors.size(); i++) {
    //     std::string module_name = sensors[i].get_info(RS2_CAMERA_INFO_NAME);
    //     if ("Tracking Module" == module_name) {
    //         ROS_INFO_STREAM("find the sensor with " << module_name);
    //         _sensor = sensors[i];
    //         break;
    //     }
    // }
    std::vector<rs2::stream_profile> stream_profiles = _sensor.get_stream_profiles();
    ROS_INFO_STREAM("there are " << stream_profiles.size() << " stream profiles in this sensor");
    _stream_profiles = stream_profiles;
    // for (int i = 0; i < stream_profiles.size(); i++) {
    //     if (stream_profiles[i].is<rs2::pose_stream_profile>()) {
    //         ROS_INFO("find the pose_stream_profile");
    //         _stream_profile = stream_profiles[i].as<rs2::pose_stream_profile>();
    //         break;
    //     }
    // }
}

void MyT265Camera::setupOdomPub() {
    std::string topic_out;
    _pnh.param("topic_odom_out", topic_out, std::string(""));
    _odom_output_pub = _nh.advertise<nav_msgs::Odometry>(topic_out, 10);

    // int pub_frequence;
    // _pnh.param("pub_frequence", pub_frequence, 50);
    // ros::Rate loop_rate(pub_frequence);

    ROS_INFO("opening stream profile...");
    _sensor.open(_stream_profiles);
    ROS_INFO("sensor open stream profile successfully");
    try {
        std::function<void(rs2::frame)> pose_callback_function;
        pose_callback_function = [this](rs2::frame frame){poseCallBack(frame);};
        ROS_INFO("starting sensor...");
        // _sensor.start([&](rs2::frame f) { ROS_INFO("rjp is a good boy"); });
        _sensor.start(pose_callback_function);
        ROS_INFO("after start function of a sensor");
    } catch(const std::exception& ex) {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    } catch(...) {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
    
}

void MyT265Camera::poseCallBack(rs2::frame frame) {
    ROS_INFO("poseCallback is running");
    _current_time = ros::Time::now();
    ROS_INFO("111111111");
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    ROS_INFO("222222222");
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    ROS_INFO("3333333333");
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = _current_time;
    msg.header.frame_id = "t265_frame";
    msg.child_frame_id = "base_link";
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;
    _tf_broad_caster.sendTransform(msg);

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
    odom_msg.header.stamp = _current_time;
    odom_msg.header.frame_id = "t265_frame";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose = pose_msg.pose;
    odom_msg.twist.twist.linear = v_msg.vector;
    odom_msg.twist.twist.angular = om_msg.vector;

    ROS_INFO("111111111");
    _odom_output_pub.publish(odom_msg);
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

void MyT265Camera::setupOdomSub() {
    if (not _use_odom_input) return;

    std::string topic_in;
    _pnh.param("topic_odom_in", topic_in, std::string(""));
    ROS_INFO_STREAM("subscribing to the topic " << topic_in);
    _odom_in_sub = _nh.subscribe(topic_in, 5, &MyT265Camera::odomCallBack, this);
    ROS_INFO("setup odom subscriber done");
}


void MyT265Camera::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("odomcallback is running");
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};
    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}

MyT265Camera::~MyT265Camera() {

}