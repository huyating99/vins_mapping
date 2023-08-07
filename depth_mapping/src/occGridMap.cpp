#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <queue>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <utility>
#include <mutex>
#include "./occGridMapUpgrade/map.h"

std::mutex m_buf;
std::queue<nav_msgs::msg::Odometry::ConstPtr> pose_buf;
std::queue<sensor_msgs::msg::LaserScan::ConstPtr> scan_buf;
nav_msgs::msg::Path::ConstPtr path = nullptr;
std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> path_buf;

Eigen::Matrix3d Ric;
double RANGE_MISS;

int MAP_SIZE_X, MAP_SIZE_Y, MAP_INIT_X, MAP_INIT_Y, NUM_RANGE_DATA;
double MAP_CELL_SIZE;
int INCREASE_PIXEL = 0;
int INCREASE_SINGLEMODE = 0;
int MISS_SCORE = 60;
double X_ROBOT_LASER, Y_ROBOT_LASER, THETA_ROBOT_LASER;
double P_OCC, P_FREE, P_PRIOR;
Eigen::Matrix3d R_c_l;
std::string MAP_SAVE_PATH;
std::string SCAN_TOPIC;
int SAVE_MAP;
// pose_graph
double time_pre = -1;
cv::Mat map_intfc_clone;

void ReadParameter(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    RANGE_MISS = fsSettings["rangeMiss"];//打空保护激光距离

    MAP_SIZE_X = fsSettings["map"]["sizex"];
    MAP_SIZE_Y = fsSettings["map"]["sizey"];
    MAP_INIT_X = fsSettings["map"]["initx"];
    MAP_INIT_Y = fsSettings["map"]["inity"];
    MAP_CELL_SIZE = fsSettings["map"]["cell_size"];
    NUM_RANGE_DATA = fsSettings["map"]["num_range_data"];
    INCREASE_PIXEL = fsSettings["map"]["increase_pixel"];
    MISS_SCORE =  fsSettings["map"]["miss_score"];
    std::cout << "MAP_SIZE_X: "  << MAP_SIZE_X << std::endl;
    std::cout << "MAP_SIZE_Y: "  << MAP_SIZE_Y << std::endl;
    std::cout << "MAP_INIT_X: "  << MAP_INIT_X << std::endl;
    std::cout << "MAP_INIT_Y: "  << MAP_INIT_Y << std::endl;
    std::cout << "MAP_CELL_SIZE: "  << MAP_CELL_SIZE << std::endl;
    std::cout << "NUM_RANGE_DATA: "  << NUM_RANGE_DATA << std::endl;

    X_ROBOT_LASER = fsSettings["robot_laser"]["x"];
    Y_ROBOT_LASER =  fsSettings["robot_laser"]["y"];
    // THETA_ROBOT_LASER = fsSettings["robot_laser"]["theta"];

    //计算 R_I_L -> theta
    cv::Mat cv_T;

    fsSettings["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    Eigen::Matrix3d eigen_R;
    eigen_R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    Ric = eigen_R;
    R_c_l << 0.0, -1.0, 0.0,
            0.0,0.0,-1.0,
            1.0,0.0,0.0;
    Eigen::Matrix3d R_I_L = Ric * R_c_l;
    Eigen::Quaterniond q_i_l(R_I_L);
    q_i_l.normalize();
    double siny_cosp = 2 * (q_i_l.w() * q_i_l.z() + q_i_l.x() * q_i_l.y());
    double cosy_cosp = 1 - 2 * (q_i_l.y() * q_i_l.y() + q_i_l.z() * q_i_l.z());
    THETA_ROBOT_LASER = -std::atan2(siny_cosp, cosy_cosp);
    // todo
    THETA_ROBOT_LASER = 1.57;//当出现scan与cam差90度时开启
    SAVE_MAP = fsSettings["save_map"];
    fsSettings["output_path"] >> MAP_SAVE_PATH;
    std::cout << "map save path: " << MAP_SAVE_PATH << "map/map.pgm"<<std::endl;

    P_OCC = fsSettings["sensor_model"]["P_occ"];
    P_FREE = fsSettings["sensor_model"]["P_free"];
    P_PRIOR = fsSettings["sensor_model"]["P_prior"];
    std::cout << "X_ROBOT_LASER: "  << X_ROBOT_LASER << std::endl;
    std::cout << "Y_ROBOT_LASER: "  << Y_ROBOT_LASER << std::endl;
    std::cout << "THETA_ROBOT_LASER: "  << THETA_ROBOT_LASER << std::endl;
    std::cout << "P_OCC: "  << P_OCC << std::endl;
    std::cout << "P_FREE: "  << P_FREE << std::endl;
    std::cout << "P_PRIOR: "  << P_PRIOR << std::endl;

    fsSettings["scan_topic"] >> SCAN_TOPIC;
    std::cout << "SCAN_TOPIC: "  << SCAN_TOPIC << std::endl;
    std::cout << "--------------------" << std::endl;
    fsSettings.release();
}

void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    // std::cout << "pose_callback: " << std::fixed << std::setprecision(6) << pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec*(1e-9)<< std::endl;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // std::cout << "scan_callback: " << std::fixed << std::setprecision(6) << scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec*(1e-9) << std::endl;
    m_buf.lock();
    scan_buf.push(scan_msg);
    m_buf.unlock();
}

void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg)
{
    // std::cout << "path_callback: " << std::fixed << std::setprecision(6) << path_msg->poses.front().header.stamp.sec + path_msg->poses.front().header.stamp.nanosec*(1e-9) << std::endl;
    m_buf.lock();
    path.reset(new nav_msgs::msg::Path);
    path = path_msg;
    // while(!path_msg->poses.empty()){
    //     auto poses = path_msg->poses;
    //     double time = poses.back().header.stamp.sec + path_msg->poses.front().header.stamp.nanosec * (1e-9);
    //     Eigen::Vector3d p = Eigen::Vector3d(
    //                         poses.front().pose.position.x,
    //                         poses.front().pose.position.y,
    //                         poses.front().pose.position.z
    //                         );
    //     Eigen::Quaterniond q = Eigen::Quaterniond(
    //                         poses.front().pose.orientation.w,
    //                         poses.front().pose.orientation.x,
    //                         poses.front().pose.orientation.y,
    //                         poses.front().pose.orientation.z
    //                         );
    //     path_buf.emplace(time, std::make_pair(p, q));
    //     path_msg->poses.pop_back();
    // }
    for(auto i = path_msg->poses.begin(); i != path_msg->poses.end(); i++){
        double time = i->header.stamp.sec + i->header.stamp.nanosec * (1e-9);
        Eigen::Vector3d p = Eigen::Vector3d(
                            i->pose.position.x,
                            i->pose.position.y,
                            i->pose.position.z
                            );
        Eigen::Quaterniond q = Eigen::Quaterniond(
                            i->pose.orientation.w,
                            i->pose.orientation.x,
                            i->pose.orientation.y,
                            i->pose.orientation.z
                            );
        path_buf.emplace(time, std::make_pair(p, q));
    }
    m_buf.unlock();
}

void OccGridmapping(mapping::ActiveSubmaps2D &upgradeMap);
int main(int argc, char **argv){
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<rclcpp::Node>("occGridMap");
    RCLCPP_INFO(node->get_logger(), "0 node_01节点已经启动.");
    
    if(argc != 2)
    {
        printf("please intput: rosrun depth_mapping occGridMap [config file] \n"
               "for example: rosrun depth_mapping occGridMap "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }

    std::string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    ReadParameter(config_file);
            
    mapping::ActiveSubmaps2D upgradeMap;
    upgradeMap.registerPub(node);

    auto sub_pose_vio         = node->create_subscription<nav_msgs::msg::Odometry>("/keyframe_pose", rclcpp::QoS(rclcpp::KeepLast(2000)), pose_callback);
    auto sub_pose_scan        = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(rclcpp::KeepLast(2000)), scan_callback);
    auto sub_path_loop        = node->create_subscription<nav_msgs::msg::Path>("/pose_graph_path", rclcpp::QoS(rclcpp::KeepLast(2000)), path_callback);
    
    std::thread process_mapping;
    process_mapping = std::thread(OccGridmapping,std::ref(upgradeMap));
    
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    return 0;
}

void OccGridmapping(mapping::ActiveSubmaps2D &upgradeMap)
{
    while(true)
    {
        m_buf.lock();
        if(pose_buf.empty() || scan_buf.empty() || path == nullptr)
        {
            // if(pose_buf.empty())
            //     std::cout << "pose_buf.empty()" <<  std::endl;
            // if(scan_buf.empty())
            //     std::cout << "scan_buf.empty()" <<  std::endl;
            // if(path == nullptr)
            //     std::cout << "path == nullptr" <<  std::endl;
            m_buf.unlock();
            continue;
        }
        auto it = path->poses.begin();
        while(it != path->poses.end() && it->header.stamp.sec + it->header.stamp.nanosec * (1e-9) <= time_pre){
            // std::cout << std::fixed << std::setprecision(6) << it->header.stamp.sec + it->header.stamp.nanosec * (1e-9) << " <= "<< time_pre <<  std::endl;
            it++;
        }
        if(it == path->poses.end()){
            m_buf.unlock();
            continue;
        }
        double pose_time = pose_buf.front()->header.stamp.sec + pose_buf.front()->header.stamp.nanosec * (1e-9);
        double scan_time = scan_buf.front()->header.stamp.sec + scan_buf.front()->header.stamp.nanosec * (1e-9);
        double path_time = it->header.stamp.sec + it->header.stamp.nanosec * (1e-9);

        // while(!scan_buf.empty() && scan_time < path_time){
        //     scan_buf.pop();
        //     scan_time = scan_buf.front()->header.stamp.sec + scan_buf.front()->header.stamp.nanosec * (1e-9);
        // }
        while(it != path->poses.end() && scan_time > path_time){
            it++;
            path_time = it->header.stamp.sec + it->header.stamp.nanosec * (1e-9);
        }
        if(it == path->poses.end()){
            // std::cout << "it != path->poses.end() && scan_time > path_time" <<  std::endl;
            m_buf.unlock();
            continue;
        }
        // std::cout << "scan_time: " << std::fixed << std::setprecision(6) << scan_time << std::endl;
        // std::cout << "pose_time: " << std::fixed << std::setprecision(6) << pose_time << std::endl;
        // std::cout << "path_time: " << std::fixed << std::setprecision(6) << path_time << std::endl;

        Eigen::Matrix<double, 7, 1> pose;
        sensor_msgs::msg::LaserScan::ConstPtr scan_ptr;
        std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> path_map_rectify;

        if(scan_time < path_time - 0.003){
            scan_buf.pop();
            std::cout << "throw old scan_ptr!" << std::endl;
            m_buf.unlock();
            continue;
        }
        else if(scan_time > path_time + 0.003){
            it++;
            std::cout << "throw old path->poses.front()!" << std::endl;
            m_buf.unlock();
            continue;
        }
        else if(pose_time < path_time - 0.003){
            pose_buf.pop();
            std::cout << "throw old pose!" << std::endl;
            m_buf.unlock();
            continue;
        }
        else if(pose_time > path_time + 0.003){
            it++;
            std::cout << "throw old path->poses.front()!" << std::endl;
            m_buf.unlock();
            continue;
        }

        std::cout << "get correct information!" << std::endl;
        std::cout << "scan_time: " << std::fixed << std::setprecision(6) << scan_time << std::endl;
        std::cout << "pose_time: " << std::fixed << std::setprecision(6) << pose_time << std::endl;
        std::cout << "path_time: " << std::fixed << std::setprecision(6) << path_time << std::endl;

        pose << pose_buf.front()->pose.pose.position.x,
                pose_buf.front()->pose.pose.position.y,
                pose_buf.front()->pose.pose.position.z,
                pose_buf.front()->pose.pose.orientation.w,
                pose_buf.front()->pose.pose.orientation.x,
                pose_buf.front()->pose.pose.orientation.y,
                pose_buf.front()->pose.pose.orientation.z;
        pose_buf.pop();
        scan_ptr = scan_buf.front();
        scan_buf.pop();
        path_map_rectify = path_buf;
        time_pre = path_time;
        m_buf.unlock();

        if(scan_ptr == nullptr){
            std::cout << "scan_ptr == nullptr! " << std::endl;
        }
        else{
            std::cout << "scan_ptr->ranges.size(): " << scan_ptr->ranges.size() <<std::endl;
        }

        upgradeMap.insertScan(scan_ptr, pose, path_time);

        if(INCREASE_PIXEL== 0){
            upgradeMap.buildLargeMap(path_map_rectify);
        }
        else{
            upgradeMap.buildLargeMapPixel(path_map_rectify);
        }
        map_intfc_clone = upgradeMap.map_intfc.clone();

        upgradeMap.publishMapToRos("world");

        std::chrono::milliseconds dura_mapping(5);
        std::this_thread::sleep_for(dura_mapping);
    }
}