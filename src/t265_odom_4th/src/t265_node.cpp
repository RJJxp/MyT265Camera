#include "../include/t265.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rjp_t265_4th_node");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    
    rs2::config cfg;
    rs2::pipeline ppl;
    rs2::pipeline_profile ppl_pf;

    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    ppl.start(cfg);
    ppl_pf = ppl.get_active_profile();
    rs2::device dev = ppl_pf.get_device();

    MyT265Pipeline t265ppl(node_handle, private_node_handle, dev, ppl);

    return 0;
}