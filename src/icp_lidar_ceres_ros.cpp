#include "icp_lidar_ceres_ros/icp_lidar_ceres_ros.h"

class CostFunctor {
    public:
    /*
        p_point: double[3], reference point
        q_point: double[3], point to be aligned
        n: double[3], normal vector of q_point
    */
    CostFunctor(const double* p_point, const double* q_point, const double* normal)
    :p_point_(p_point[0], p_point[1]), q_point_(q_point[0], q_point[1]), normal_(normal[0], normal[1]) {}
    /*
        Template param T: double[]
        x: double[3], information of R,t formed by (x,y,theta)
        residual: double, error value
    */
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        Eigen::Matrix<T, 2, 2> R; 
        const T cos = ceres::cos(x[2]);
        const T sin = ceres::sin(x[2]);
        R << cos, -sin,
            sin, cos;
        Eigen::Matrix<T, 2, 1> t(x[0], x[1]);
        residual[0] = normal_.transpose() * (R * p_point_ + t - q_point_);
        return true;
    }
    private:
    Vector2d p_point_, q_point_, normal_;
};

class CostFunctor_p2p {
    public:
    CostFunctor_p2p(const double* p_point, const double* q_point)
    :p_point_(p_point[0], p_point[1]), q_point_(q_point[0], q_point[1]) {}

    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        Eigen::Matrix<T, 2, 2> R; 
        const T cos = ceres::cos(x[2]);
        const T sin = ceres::sin(x[2]);
        R << cos, -sin,
            sin, cos;
        Eigen::Matrix<T, 2, 1> t(x[0], x[1]);
        auto res = R * p_point_ + t - q_point_;
        residual[0] = res[0];
        residual[1] = res[1];
        return true;
    }
    private:
    Vector2d p_point_, q_point_;
};

icp_lidar_ceres_ros::icp_lidar_ceres_ros(): scan1(false), scan2(false) {
    scan1_sub_ = nh_.subscribe("scan1", 1, &icp_lidar_ceres_ros::Scan1Callback, this);
    scan2_sub_ = nh_.subscribe("scan2", 1, &icp_lidar_ceres_ros::Scan2Callback, this);
    aligned_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
}

void icp_lidar_ceres_ros::Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    reference_points.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    angle_max_ = msg->angle_max;
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;
    range_min_ = msg->range_min;
    range_max_ = msg->range_max;
    int row = 0;
    for(int i = 0; i < ranges.size(); i++) {
        if(ranges[i] == 0) continue;
        push_back_(reference_points, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
        row++;
    }
    reference_points.transposeInPlace();
    scan1 = true;
}

void icp_lidar_ceres_ros::Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    points_to_be_aligned.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    int row = 0;
    for(int i = 0; i < ranges.size(); i++) {
        if(ranges[i] == 0) continue;
        push_back_(points_to_be_aligned, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
        row++;
    }
    points_to_be_aligned.transposeInPlace();
    scan2 = true;
}

void icp_lidar_ceres_ros::knn_kdtree(const MatrixXd reference_points, const MatrixXd points_to_be_aligned) {
    knncpp::KDTreeMinkowskiX<double, knncpp::EuclideanDistance<double>> kdtree(reference_points);

    kdtree.setBucketSize(1);
    kdtree.build();

    kdtree.query(points_to_be_aligned, 1, indices_, distances_);
}

MatrixXd icp_lidar_ceres_ros::icp_non_linear(const MatrixXd& reference_points, const MatrixXd& points, int max_iterations, double distance_threshold,
        int point_pairs_threshold, bool verbose) {
    std::vector<Vector2d> normals = compute_normals_(reference_points);
    double x[3] = {0,};
    MatrixXd result_points = points;

    for(int iter_num = 0; iter_num < max_iterations; iter_num++) {
        knn_kdtree(reference_points, result_points);

        MatrixXd points_pair_a(1,2), points_pair_b(1,2);
        int nn_index = 0;
        for(int i = 0; i < distances_.size(); i++) {
            if(distances_(nn_index) < distance_threshold) {
                push_back_(points_pair_a, result_points.block<2,1>(0,nn_index), nn_index);
                push_back_(points_pair_b, reference_points.block<2,1>(0,indices_(nn_index)), nn_index);
                nn_index++;
            }
        }
        

        double** p_points = pointsMatrix_to_2darray_(points_pair_a);
        double** q_points = pointsMatrix_to_2darray_(points_pair_b);

        Problem problem;
        /*
            Template of AutoDiffCostFunction
            First number of <CostFunctor, 1, 3> means size of residual
            numbers after first one means sizes of variable(double array) that compose cost function of optimization
            i.e. <CostFunctor, 1, 3> <=> {size of residual, size of x}
        */
        for(int i = 0; i < points_pair_a.rows(); i++) {
            double normal[2] = {normals[i][0], normals[i][1]};
            CostFunction *cost_function
                = new AutoDiffCostFunction<CostFunctor, 1, 3>(new CostFunctor(p_points[i], q_points[i], normal));
                // = new AutoDiffCostFunction<CostFunctor_p2p, 2, 3>(new CostFunctor_p2p(p_points[i], q_points[i]));
            problem.AddResidualBlock(cost_function, NULL, x);
        }

        Solver::Options options;
        // options.minimizer_progress_to_stdout = true;
        Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if(verbose) {
            std::cout << summary.BriefReport();
            std::cout << "x: ";
            for(int i=0; i<3; i++) std::cout << x[i] << ' ';
            std::cout << "\n";
        }

        Matrix2d R;
        R << ceres::cos(x[2]), -ceres::sin(x[2]),
            ceres::sin(x[2]), ceres::cos(x[2]);
        MatrixXd t(2, result_points.cols());
        t << MatrixXd::Constant(1,result_points.cols(),x[0]), MatrixXd::Constant(1,result_points.cols(),x[1]);
        result_points = R * result_points + t;
    }
    return result_points;
}

void icp_lidar_ceres_ros::run() {
    MatrixXd aligned_points;
    sensor_msgs::LaserScan result_laserscan;
    ros::Rate rate(25);
    while(ros::ok()) {
        if(scan1 && scan2) {
            // cout << "Publishing aligned scan topic (/scan)" << endl;
            int n = (angle_max_-angle_min_) / angle_increment_;
            aligned_points = icp_non_linear(reference_points, points_to_be_aligned, 10, 10);
            std::vector<float> ranges(n);
            for(int i = 0; i < aligned_points.cols() ; i++) {
                double angle = atan2(aligned_points(1,i), aligned_points(0,i));
                int index = (-angle_min_ - angle) / angle_increment_;
                ranges[index] = sqrt(pow(aligned_points(0,i), 2) + pow(aligned_points(1,i), 2));
            }
            result_laserscan.header.frame_id = "laser1";
            result_laserscan.angle_min = angle_min_;
            result_laserscan.angle_max = angle_max_;
            result_laserscan.angle_increment = angle_increment_;
            result_laserscan.range_min = range_min_;
            result_laserscan.range_max = range_max_;
            result_laserscan.ranges = ranges;
            aligned_scan_pub_.publish(result_laserscan); // scan publish
            scan1 = false; scan2 = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/*
    reference_point: (x,y)
*/
std::vector<Vector2d> icp_lidar_ceres_ros::compute_normals_(const MatrixXd& reference_points) {
    std::vector<Vector2d> normals;
    normals.push_back(Vector2d(0,0));
    Vector2d normal;
    for(int i = 1; i < reference_points.cols()-1; i++) {
        double dx = reference_points(0,i+1) - reference_points(0,i-1);
        double dy = reference_points(1,i+1) - reference_points(1,i-1);
        
        normal << -dy, dx;
        normal = normal/normal.norm();
        normals.push_back(normal);
    }
    normals.push_back(Vector2d(0,0));
    return normals;
}

Vector2d icp_lidar_ceres_ros::split_(string input, char delimiter) {
    Vector2d answer;
    stringstream ss(input);
    string temp;

    for(int i = 0; getline(ss, temp, delimiter); i++) {
        answer(i) = stod(temp);
    }
    return answer;
}

void icp_lidar_ceres_ros::push_back_(MatrixXd& m, Vector2d&& values, std::size_t row) {
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}

double** icp_lidar_ceres_ros::pointsMatrix_to_2darray_(const MatrixXd& points) {
    double** array_2d = new double*[points.rows()];
    for(int i = 0; i < points.rows(); i++) {
        array_2d[i] = new double[2];
    }
    for(int i=0; i<points.rows(); i++) {
        for(int j=0; j<2; j++) {
            array_2d[i][j] = points(i,j);
        }
    }
    return array_2d;
}