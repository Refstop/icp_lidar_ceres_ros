#include "icp_lidar_ceres_ros/icp_lidar_ceres_ros.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "icp_lidar_ceres");
    icp_lidar_ceres_ros icp;
    icp.run();
    return 0;
}