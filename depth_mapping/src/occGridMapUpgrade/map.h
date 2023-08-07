//
// 子地图1.0版本
// 包含：回环rectify_path子地图
// 不包含：已有先验地图的情况下建图buildLargeMapOnPrior等2023/05/21的master分支内容

#ifndef BETA_MAP_H
#define BETA_MAP_H

// #include "../utility/parameters.h"
// #include "../common/sensorMsg.h"
#include "submap.h"
#include "submap_limit.h"
#include "pose2d_.h"
#include <thread>
#include <mutex>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

extern double RANGE_MISS;
extern int NUM_RANGE_DATA;
extern double MAP_CELL_SIZE;
extern double X_ROBOT_LASER, Y_ROBOT_LASER, THETA_ROBOT_LASER;
extern int INCREASE_PIXEL;
extern int MISS_SCORE;
extern std::string MAP_SAVE_PATH;
extern int SAVE_MAP;
extern double P_OCC, P_FREE, P_PRIOR;


/**
 * @brief 有2个活跃的子图,其它的已经构建完成
 * 只有初始化时才只有1个子图.
 */
namespace mapping{
    // extern ros::Publisher mapPublisher;
    extern rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;

    class ActiveSubmaps2D
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       explicit ActiveSubmaps2D(int num_range_data);
       ActiveSubmaps2D();
        ~ActiveSubmaps2D(){};

        // void convertLaserScanToRangeData(RangeData& range_data,Scan_MSGConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan);
        void convertLaserScanToRangeData(RangeData& range_data, sensor_msgs::msg::LaserScan::ConstPtr Scan, Eigen::Matrix<double, 7, 1> pose_scan);
        // void convertLaserScanToRangeDataV2(RangeData& range_data,Scan_MSGConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan);
        void convertLaserScanToRangeDataV2(RangeData& range_data,sensor_msgs::msg::LaserScan::ConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan);
        // void insertScan(Scan_MSGConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan, double header);
        void insertScan(sensor_msgs::msg::LaserScan::ConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan, double header);
        std::vector<std::pair<double,std::shared_ptr<Submap>>> InsertRangeData(const RangeData& range_data,const double header);

        std::vector<std::pair<double,std::shared_ptr<Submap>>> submaps() const;

        void buildLargeMap(std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> rectify_path);
        void buildLargeMapPixel(std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> rectify_path); 
        void buildLargeMapPixelV2(); 
        void saveMap();

        void publishMapToRos(const std::string& frame_id);
        void registerPub(rclcpp::Node::SharedPtr node);

    public:
        MapLimits m_mapLimit;//总的大地图相关信息
        cv::Mat m_map,m_map_clone;
        cv::Mat map_intfc;
        Eigen::Vector2i cell_start;//给PAC专用的起点
        std::thread m_saveMap;
        std::mutex map_mute;
        bool map_finish;     
        float m_resolution;   


    private:
        std::unique_ptr<ProbabilityGridRangeDateInseter> CreateRangeDataInserter();
        std::unique_ptr<ProbabilityGrid> CreateGrid(const Eigen::Vector3f& origin);
        void AddSubmap(const Eigen::Vector3f& origin,const double& header);


        std::vector<std::pair<double,std::shared_ptr<Submap>>> submaps_;
        std::unique_ptr<ProbabilityGridRangeDateInseter> range_data_inserter_;
        ValueConversionTables conversion_tables_;

        int num_range_data;
        int increase_pixel;
        
        float range_miss;
        mapping::Pose2d T_r_l_;
    };
}


#endif //BETA_MAP_H
