#ifndef ICP_LIDAR_CERES_ROS
#define ICP_LIDAR_CERES_ROS

#include <iostream>
#include <icp_lidar_ceres_ros/knncpp.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#define RAD2DEG(rad) rad*(180/M_PI)
#define DEG2RAD(deg) deg*(M_PI/180)

using namespace std;
using namespace Eigen;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

typedef knncpp::Matrixi Matrixi;

class icp_lidar_ceres_ros {
    public:
    MatrixXd reference_points, points_to_be_aligned;
    icp_lidar_ceres_ros();
    ~icp_lidar_ceres_ros() {}
    void Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // reference_points
    void Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // points_to_be_aligned
    void knn_kdtree(const MatrixXd reference_points, const MatrixXd points_to_be_aligned);
    // point to plane icp
    MatrixXd icp_non_linear(const MatrixXd& reference_points, const MatrixXd& points, int max_iterations = 10, double distance_threshold = 0.3,
        int point_pairs_threshold=8, bool verbose=true);
    void run();

    private:
    Matrixi indices_;
    MatrixXd distances_;
    ros::NodeHandle nh_;
    ros::Subscriber scan1_sub_, scan2_sub_;
    ros::Publisher aligned_scan_pub_;
    float angle_max_, angle_min_, angle_increment_, range_min_, range_max_;
    bool scan1, scan2;
    std::vector<Vector2d> compute_normals_(const MatrixXd& reference_points);
    Vector2d split_(string input, char delimiter);
    void push_back_(MatrixXd& m, Vector2d&& values, std::size_t row);
    double** pointsMatrix_to_2darray_(const MatrixXd& points);
};

#endif