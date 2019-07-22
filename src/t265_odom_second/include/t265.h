#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

class MyT265Camera {
public:
    MyT265Camera(ros::NodeHandle& node_hanle,
                 ros::NodeHandle& private_node_handle,
                 std::string serial_no);
    ~MyT265Camera();

private:
    rs2::device getDevice();
    void setupDevice();
    void setupOdomPub();
    void poseCallBack(rs2::frame frame);
    
    void setupOdomSub();
    void initialOdomInput();
    void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

private:
    std::string _serial_no;
    ros::NodeHandle& _nh, _pnh;
    ros::Subscriber _odom_in_sub;
    ros::Publisher _odom_output_pub;
    ros::Time _current_time;
    tf::TransformBroadcaster _tf_broad_caster;
    
    rs2::device _device;
    rs2::sensor _sensor;
    rs2::stream_profile _stream_profile;
    rs2::wheel_odometer _wo_snr;
    bool _use_odom_input;
    
    std::vector<rs2::stream_profile> _stream_profiles;
    



};