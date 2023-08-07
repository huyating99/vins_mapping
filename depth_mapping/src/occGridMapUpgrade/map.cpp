//
// Created by lao-tang on 2022/5/24.
//

#include "./map.h"
#include <mutex>
#include <cmath>

#include <boost/format.hpp>

#include <condition_variable>//YH230704

namespace mapping{

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;

/**
 * @param[in] num_range_data 一个子图由多少帧点云构成
 */
    ActiveSubmaps2D::ActiveSubmaps2D(int num_range_data):num_range_data(num_range_data),
    range_data_inserter_(CreateRangeDataInserter()),m_mapLimit(MapLimits(MAP_CELL_SIZE,{0,0},{0,0},CellLimits(1,1))) {
        std::cout << "mapping MAP_CELL_SIZE: " << MAP_CELL_SIZE << std::endl;
        std::cout << "mapping X_ROBOT_LASER: " << X_ROBOT_LASER << std::endl;
        std::cout << "mapping Y_ROBOT_LASER: " << Y_ROBOT_LASER << std::endl;
        std::cout << "mapping THETA_ROBOT_LASER: " << THETA_ROBOT_LASER << std::endl;
        std::cout << "mapping num_range_data: " << num_range_data << std::endl;
        m_resolution = MAP_CELL_SIZE;
        T_r_l_  = mapping::Pose2d(X_ROBOT_LASER, Y_ROBOT_LASER, THETA_ROBOT_LASER);
        map_finish = false;

    }

    ActiveSubmaps2D::ActiveSubmaps2D():range_data_inserter_(CreateRangeDataInserter()),m_mapLimit(MapLimits(MAP_CELL_SIZE,{0,0},{0,0},CellLimits(1,1))) {
        std::cout << "---------------------------------------------" << std::endl;
        std::cout << "mapping MAP_CELL_SIZE" << MAP_CELL_SIZE << std::endl;
        std::cout << "mapping X_ROBOT_LASER" << X_ROBOT_LASER << std::endl;
        std::cout << "mapping Y_ROBOT_LASER" << Y_ROBOT_LASER << std::endl;
        std::cout << "mapping THETA_ROBOT_LASER" << THETA_ROBOT_LASER << std::endl;
        num_range_data = NUM_RANGE_DATA;
        increase_pixel = INCREASE_PIXEL;
        range_miss = RANGE_MISS;
        std::cout << "INCREASE_PIXEL" << increase_pixel << std::endl;
        m_resolution = MAP_CELL_SIZE;
        T_r_l_  = mapping::Pose2d(X_ROBOT_LASER, Y_ROBOT_LASER, THETA_ROBOT_LASER);
        cell_start << 1 , 1;
        m_saveMap= std::thread(&ActiveSubmaps2D::saveMap,this);
        std::cout << "ActiveSubmaps2D::ActiveSubmaps2D() finished!" << std::endl;
        map_finish = false;
    }

    // 返回指向 Submap2D 的 shared_ptr指针 的vector
    std::vector<std::pair<double,std::shared_ptr<Submap>>> ActiveSubmaps2D::submaps() const
    {
        return std::vector<std::pair<double,std::shared_ptr<Submap>>>(submaps_.begin(),submaps_.end());
    }

    // 将点云数据写入到submap中
    std::vector<std::pair<double,std::shared_ptr<Submap>>> ActiveSubmaps2D::InsertRangeData(const RangeData& range_data,double header) {
        // 如果第二个子图插入节点的数据等于num_range_data时,就新建个子图
        // 因为这时第一个子图应该已经处于完成状态了
        if (submaps_.empty() ||
            submaps_.back().second->num_range_data() == num_range_data) {
            AddSubmap(range_data.origin,header);
        }
//        // 将一帧雷达数据同时写入后两个子图中todo
//        for (auto& submap : submaps_) {
//            submap->InsertRangeData(range_data, range_data_inserter_.get());
//        }
// 0704 huadd
//        if(submaps_.size()>=2){
//            int size = submaps_.size();
//            //将雷达数据写到栅格地图中
//            submaps_[size-1].second->InsertRangeData(range_data, range_data_inserter_.get());
//            submaps_[size-2].second->InsertRangeData(range_data, range_data_inserter_.get());
//            // 第一个子图的节点数量等于2倍的num_range_data时,第二个子图节点数量应该等于num_range_data
//            if(submaps_[size-2].second->num_range_data() == 2 * num_range_data){
//                submaps_[size-2].second->Finish();
//                cv::Mat showSubmap;
//                submaps_[size-2].second->grid()->ProbabilityGridPicture(showSubmap);
//                std::cout << "save submap path" << MAP_RESULT_PATH+"/"+"map/" << std::endl;
//                cv::imwrite(MAP_RESULT_PATH+"/"+"map/"+std::to_string(size-2)+".png",showSubmap);
//                cv::imwrite(MAP_RESULT_PATH+"/"+"map/"+std::to_string(size-2)+".jpg",showSubmap);
//            }


//        }else{
            int size = submaps_.size();
            submaps_[size-1].second->InsertRangeData(range_data, range_data_inserter_.get());
//        }
            std::cout << "Submap's numbers is " << int(submaps_.size()) << std::endl;
//        // 第一个子图的节点数量等于2倍的num_range_data时,第二个子图节点数量应该等于num_range_data
//        if (submaps_.front()->num_range_data() == 2 * num_range_data) {
//            submaps_.front()->Finish();
//        }
        return submaps();
    }

    // 创建地图数据写入器
    std::unique_ptr<ProbabilityGridRangeDateInseter> ActiveSubmaps2D::CreateRangeDataInserter()
    {
            return std::make_unique<ProbabilityGridRangeDateInseter>();
    }

    // 以当前雷达原点为地图原件创建地图
    std::unique_ptr<ProbabilityGrid> ActiveSubmaps2D::CreateGrid(const Eigen::Vector3f& origin) {
        // 地图初始大小,100个栅格
        constexpr int kInitialSubmapSize = 100;
        float resolution = m_resolution; // param: grid_options_2d.resolution

            // 概率栅格地图
            return std::make_unique<ProbabilityGrid>(
                        MapLimits(resolution,
                                // 左上角坐标为坐标系的最大值, origin位于地图的中间
                                  origin.cast<double>().head<2>() + 0.5 * kInitialSubmapSize *
                                                          resolution *
                                                          Eigen::Vector2d::Ones(),
                                    origin.cast<double>().head<2>() + 0.5 * kInitialSubmapSize *
                                                          -1 * resolution *
                                                          Eigen::Vector2d::Ones(),
                                  CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
                        //CorrespondenceCost表示的是被珊格没有被占据的概率
                        kMinCorrespondenceCost,
                        kMaxCorrespondenceCost,
                        &conversion_tables_);
        }//YH230704，新增中间-1项表示min

    // 新增一个子图,根据子图个数判断是否删掉第一个子图
    void ActiveSubmaps2D::AddSubmap(const Eigen::Vector3f& origin,const double& header) {
        // 调用AddSubmap时第一个子图一定是完成状态,所以子图数为2时就可以删掉第一个子图了
//        if (submaps_.size() >= 2) {
//            // This will crop the finished Submap before inserting a new Submap to
//            // reduce peak memory usage a bit.
//            ICHECK(submaps_.front()->insertion_finished());
//            // 删掉第一个子图的指针
//            submaps_.erase(submaps_.begin());
//        }
        // 新建一个子图, 并保存指向新子图的智能指针
        submaps_.push_back(std::make_pair(header,std::make_unique<Submap>(
                origin,
                std::unique_ptr<ProbabilityGrid>(
                        static_cast<ProbabilityGrid*>(CreateGrid(origin).release())),
                &conversion_tables_)));
    }

    void ActiveSubmaps2D::convertLaserScanToRangeData(RangeData& range_data,sensor_msgs::msg::LaserScan::ConstPtr scan_Own,
                                     Eigen::Matrix<double, 7, 1> pose_scan){
        const double ang_min = scan_Own->angle_min;
        const double ang_max = scan_Own->angle_max;
        const double ang_inc = scan_Own->angle_increment;
        const double range_max = scan_Own->range_max;
        const double range_min = scan_Own->range_min;

        // TODO 得到匹配的数据 位姿：pose_scan  SCAN：scan_msg_cptr
        double x = pose_scan[0];
        double y = pose_scan[1];
        // double theta = tf::getYaw ( odom.pose.pose.orientation );
        // cout << "theta from TF: " << theta << endl;
        // Eigen::Quaterniond q_tmp(pose_scan[3],pose_scan[4],pose_scan[5],pose_scan[6]);
        // Eigen::Vector3d yaw = Utility::R2ypr(q_tmp.normalized().toRotationMatrix());
        // cout << "theta from R2ypr:  " << yaw << endl;
        //这个公式是按照外部旋转方式得到
        double siny_cosp = 2 * (pose_scan[3] * pose_scan[6] + pose_scan[4] * pose_scan[5]);
        double cosy_cosp = 1 - 2 * (pose_scan[5] * pose_scan[5] + pose_scan[6] * pose_scan[6]);
        double angle = std::atan2(siny_cosp, cosy_cosp);
        // cout << "theta from atan2:  " << angle << endl;

        Pose2d robot_pose = Pose2d( x, y, angle);

        /* 转换到世界坐标系下 */
        Pose2d laser_pose = robot_pose * T_r_l_;
        //为激光坐标系到世界坐标系下的变换
        range_data.origin<<laser_pose.x_,laser_pose.y_,laser_pose.theta_;
        /* for every laser beam */
        for(size_t i = 0; i < scan_Own->ranges.size(); i ++)
        {
            /* 获取当前beam的距离 */
            if(std::isnan(scan_Own->ranges.at(i)))
            {
                continue;
            }
            float R = scan_Own->ranges.at(i);

            if((R > range_max) || (R < range_min))
                continue;

            double angle = ang_inc * i + ang_min;
            if(angle>ang_max){
                std::cout << "点云格式出错" << std::endl;
                throw;
            }
            double cangle = cos(angle);
            double sangle = sin(angle);

            Eigen::Vector2d p_l(R * cangle,R * sangle); //在激光雷达坐标系下的坐标

            Eigen::Vector2d p_w = laser_pose * p_l;
            sensor::PointCloud::PointType Pw;
            Pw.position << p_w[0],p_w[1],0;
            range_data.returns.push_back(Pw);
        }

    }
    
    void ActiveSubmaps2D::convertLaserScanToRangeDataV2(RangeData& range_data,sensor_msgs::msg::LaserScan::ConstPtr scan_Own,
                                     Eigen::Matrix<double, 7, 1> pose_scan){
        const double ang_min = scan_Own->angle_min;
        const double ang_max = scan_Own->angle_max;
        const double ang_inc = scan_Own->angle_increment;
        const double range_max = scan_Own->range_max;
        const double range_min = scan_Own->range_min;

        // TODO 得到匹配的数据 位姿：pose_scan  SCAN：scan_msg_cptr
        double x = pose_scan[0];
        double y = pose_scan[1];
        // double theta = tf::getYaw ( odom.pose.pose.orientation );
        // cout << "theta from TF: " << theta << endl;
        // Eigen::Quaterniond q_tmp(pose_scan[3],pose_scan[4],pose_scan[5],pose_scan[6]);
        // Eigen::Vector3d yaw = Utility::R2ypr(q_tmp.normalized().toRotationMatrix());
        // cout << "theta from R2ypr:  " << yaw << endl;
        //这个公式是按照外部旋转方式得到
        double siny_cosp = 2 * (pose_scan[3] * pose_scan[6] + pose_scan[4] * pose_scan[5]);
        double cosy_cosp = 1 - 2 * (pose_scan[5] * pose_scan[5] + pose_scan[6] * pose_scan[6]);
        double angle = std::atan2(siny_cosp, cosy_cosp);
        // cout << "theta from atan2:  " << angle << endl;

        Pose2d robot_pose = Pose2d(x, y, angle);

        /* 转换到世界坐标系下 */
        Pose2d laser_pose = robot_pose * T_r_l_;

        //pose_graph
        range_data.origin << laser_pose.x_,laser_pose.y_,laser_pose.theta_;
        /* for every laser beam */
        for(size_t i = 0; i < scan_Own->ranges.size(); i ++)
        {
            /* 获取当前beam的距离 */
            if(std::isnan(scan_Own->ranges.at(i)))
            {
                continue;
            }
            float R = scan_Own->ranges.at(i);

            if(((R > range_max) || (R < range_min))&&(R != 0))
                continue;

            double angle = ang_inc * i + ang_min;
            if(angle > ang_max){
                std::cerr << "点云格式出错" << std::endl;
                throw;
            }
            double cangle = cos(angle);
            double sangle = sin(angle);

            bool hit_status = 1;
            if(R == 0){
                R = range_miss;
                hit_status = 0;//表示这点实际不存在,给他一个range_miss大小的扫描
            }

            Eigen::Vector2d p_l(R * cangle,R * sangle); //在激光雷达坐标系下的坐标

            Eigen::Vector2d p_w = laser_pose * p_l;
            sensor::PointCloud::PointType Pw;
            Pw.position << p_w[0],p_w[1],0;
            Pw.status = hit_status;//YH230705
            range_data.returns.push_back(Pw);

        }

    }

    void ActiveSubmaps2D::buildLargeMap(std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> rectify_path){
        Eigen::AlignedBox2d map_scope;
        Eigen::AlignedBox2d submap_scope;

        for(auto submap:submaps_){
            double key = submap.first;
            auto iter_pose = rectify_path.find(key);
            if(iter_pose==rectify_path.end()){
                std::cout << "can't find correct pose in rectify_path" << std::endl;
                throw;
            }

            MapLimits limits = submap.second->grid()->limits();
            transform::Rigid3d submap_Pose = submap.second->local_pose();
            Eigen::Vector2d max_ = limits.max();
            //std::cout << "before rectify pose max x y =" << max_.x() << "      " << max_.y() << std::endl;
            Eigen::Vector3d correct_max,correct_min;
            correct_max << max_.x(),max_.y(),0.0;//算出了以前的点在世界坐标系下的值
            Eigen::Matrix3d submap_R,correctsubmap_R;
            Eigen::Vector3d submap_t,correctsubmap_t;
            submap_R = submap_Pose.rotation().toRotationMatrix();//为子图对应的虚拟激光坐标系在全局坐标系下的Ｒ
            submap_t = submap_Pose.translation();//为子图对应的虚拟激光坐标系在全局坐标系下的t
            correctsubmap_t = iter_pose->second.first;//为子图对应的imu坐标系在全局坐标系下的Ｒ
            correctsubmap_R = iter_pose->second.second.toRotationMatrix();

            Eigen::Quaterniond correctsubmap_q;
            correctsubmap_q = iter_pose->second.second;
            double siny_cosp = 2 * (correctsubmap_q.w() * correctsubmap_q.z() + correctsubmap_q.x() * correctsubmap_q.y());
            double cosy_cosp = 1 - 2 * (correctsubmap_q.y() * correctsubmap_q.y()+ correctsubmap_q.z() * correctsubmap_q.z());
            double angle = std::atan2(siny_cosp, cosy_cosp);
            // cout << "theta from atan2:  " << angle << endl;

            Pose2d robot_pose = Pose2d( correctsubmap_t[0], correctsubmap_t[1], angle);

            //* 转换到世界坐标系下 */
            // 激光在世界坐标系 = 机器人在世界坐标系 * 激光在机器人坐标系
            Pose2d laser_pose = robot_pose * T_r_l_;//得到修正后的激光坐标系在世界坐标系下的相对变换

            transform::Rigid3d correctsubmap_l(Eigen::Vector3d(laser_pose.x_, laser_pose.y_, 0.0),
                          Eigen::Quaterniond(Eigen::AngleAxisd(laser_pose.theta_,Eigen::Vector3d::UnitZ())));////得到修正后的激光坐标系在世界坐标系下的相对变换
            correctsubmap_R = correctsubmap_l.rotation().toRotationMatrix();
            correctsubmap_t = correctsubmap_l.translation();

            correct_max = submap_R.transpose()*correct_max - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
            correct_max = correctsubmap_R * correct_max + correctsubmap_t;//经过修正后的在世界坐标系下的值
            max_ << correct_max.x(),correct_max.y();
           //std::cout << "after rectify pose max x y =" << max_.x() << "     " << max_.y() << std::endl;
            CellLimits cell_limits_ ;
            cell_limits_ = limits.cell_limits();

            Eigen::Vector2d min_;
            Eigen::Vector2f min;
            //对应的是栅格上右下角对应的世界坐标值
            min = limits.GetCellCenter({cell_limits_.num_x_cells,cell_limits_.num_y_cells});
            //std::cout << "before rectify pose min x y =" << min.x() << "     " << min.y() << std::endl;
            correct_min << min.x(),min.y(),0.0;
            correct_min = submap_R.transpose()*correct_min - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
            correct_min = correctsubmap_R * correct_min + correctsubmap_t;//经过修正后的在世界坐标系下的值
            min_ << correct_min.x(),correct_min.y();
           //std::cout << "after rectify pose min x y =" << min_.x() << "     " << min_.y() << std::endl;
            map_scope.extend(max_);
            map_scope.extend(min_);
        }


        Eigen::Array2d Colrow =  (map_scope.max()-map_scope.min())/m_resolution;

        Eigen::Vector2f max, min;
        max << map_scope.max().x(),map_scope.max().y();
        min << map_scope.min().x(),map_scope.min().y();

        //根据子图统计出子图所占据的在世界坐标系下的最大值和最小值，判断这个最大值或最小值是否被m_mapLimit所包含，若不包含则将地图扩大
        while((!m_mapLimit.Contains(m_mapLimit.GetCellIndex(max)))||
        (!m_mapLimit.Contains(m_mapLimit.GetCellIndex(min)))){
            m_map = cv::Mat(m_mapLimit.cell_limits().num_x_cells*2,
                            m_mapLimit.cell_limits().num_y_cells*2,CV_64FC1,cv::Scalar(0.5));
            const float x_offset = float(m_mapLimit.cell_limits().num_x_cells) / 2.0;
            const float y_offset = float(m_mapLimit.cell_limits().num_y_cells) / 2.0;
            Eigen::Array2d max;
            // max << map_scope.max().x()+y_offset*m_resolution,
            // map_scope.max().y()+x_offset*m_resolution;//会导致不在中心
            max << m_mapLimit.max().x()+y_offset*m_resolution,
            m_mapLimit.max().y()+x_offset*m_resolution;
            std::cout << "max x,y=------------------" << max.x() << "     " << max.y() << std::endl;
            CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells*2,
                                       m_mapLimit.cell_limits().num_y_cells*2) ;
            MapLimits mapLimits(m_resolution,max,-max,mapCell_limits_);//YH230704新增min
            map_mute.lock();
            m_mapLimit = mapLimits;
            map_mute.unlock();
            map_finish = false;
        }

        m_map.setTo(cv::Scalar(0.5));
        //std::cout << m_map << std::endl;
        for(auto submap:submaps_){
            Eigen::Array2i offset;
            CellLimits cell_limits;

            auto grid = submap.second->grid();
            double key = submap.first;

            auto iter_pose = rectify_path.find(key);
            if(iter_pose==rectify_path.end()){
                std::cout << "can't find correct pose in rectify_path" << std::endl;
                throw;
            }
            transform::Rigid3d submap_Pose = submap.second->local_pose();
            Eigen::Matrix3d submap_R,correctsubmap_R;
            Eigen::Vector3d submap_t,correctsubmap_t;
            submap_R = submap_Pose.rotation().toRotationMatrix();//得到的是子图的坐标
            submap_t = submap_Pose.translation();
            correctsubmap_t = iter_pose->second.first;//得到子图经过回环修正后的坐标
            correctsubmap_R = iter_pose->second.second.toRotationMatrix();

              Eigen::Quaterniond correctsubmap_q;
            correctsubmap_q = iter_pose->second.second;
            double siny_cosp = 2 * (correctsubmap_q.w() * correctsubmap_q.z() + correctsubmap_q.x() * correctsubmap_q.y());
            double cosy_cosp = 1 - 2 * (correctsubmap_q.y() * correctsubmap_q.y()+ correctsubmap_q.z() * correctsubmap_q.z());
            double angle = std::atan2(siny_cosp, cosy_cosp);
            // cout  <<  "theta from atan2:  "  <<  angle  <<  endl;

            Pose2d robot_pose = Pose2d( correctsubmap_t[0], correctsubmap_t[1], angle);

             //* 转换到世界坐标系下 */
            Pose2d laser_pose = robot_pose * T_r_l_;//得到修正后的激光坐标系在世界坐标系下的相对变换

            transform::Rigid3d correctsubmap_l(Eigen::Vector3d(laser_pose.x_, laser_pose.y_, 0.0),
                          Eigen::Quaterniond(Eigen::AngleAxisd(laser_pose.theta_,Eigen::Vector3d::UnitZ())));////得到修正后的激光坐标系在世界坐标系下的相对变换
            correctsubmap_R = correctsubmap_l.rotation().toRotationMatrix();
            correctsubmap_t = correctsubmap_l.translation();

            MapLimits limit = grid->limits();

            //计算地图有效区域
            grid->ComputeCroppedLimits(&offset, &cell_limits);//offset x对应的是行,cell_limits.num_x_cells也是对应的行

            Eigen::Array2i newCell_index;
            Eigen::Vector2f point_word;
            Eigen::Vector2f correct_pointWorld2f;
            Eigen::Vector3d correct_pointWorld;
            // 遍历地图, 将栅格数据存入cells，xy_index的x先加//xy_index对应的也是行列
            for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
                if (!grid->IsKnown(xy_index + offset)) {
                    continue;
                }
                float delta =grid->GetProbability(xy_index + offset);

                point_word = limit.GetCellCenter(xy_index + offset); //得到子地图下的栅格坐标在未修正的全局地图上的坐标
                correct_pointWorld << point_word.x(),point_word.y(),0;//算出了以前的点在世界坐标系下的值
                correct_pointWorld = submap_R.transpose()*correct_pointWorld - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
                //激光在真实世界坐标系下
                correct_pointWorld = correctsubmap_R * correct_pointWorld + correctsubmap_t;//经过修正后的在世界坐标系下的值
                correct_pointWorld2f << correct_pointWorld[0],correct_pointWorld[1];
                newCell_index = m_mapLimit.GetCellIndex(correct_pointWorld2f);//得到修正后的世界坐标系下的值在全局坐标系下的珊格坐标
                if(!m_mapLimit.Contains(m_mapLimit.GetCellIndex(correct_pointWorld2f))){
                    std::cout << "point beyond the scope of map" << std::endl;
                    throw ;
                }
                m_map.at<double>(newCell_index.x(), newCell_index.y())=delta;//第一个参数是行第二个是列
            }
        }
        map_finish = true;

    }
   
   //YH230613
    void ActiveSubmaps2D::buildLargeMapPixel(std::unordered_map<double,std::pair<Eigen::Vector3d,Eigen::Quaterniond>> rectify_path){
        Eigen::AlignedBox2d map_scope;
        Eigen::AlignedBox2d submap_scope;

        for(auto submap:submaps_){
            //pose_graph
            double key = submap.first;
            auto iter_pose = rectify_path.find(key);
            if(iter_pose==rectify_path.end()){
                std::cout << "can't find correct pose in rectify_path" << std::endl;
                throw;
            }

            MapLimits limits = submap.second->grid()->limits();
            transform::Rigid3d submap_Pose = submap.second->local_pose();
            Eigen::Vector2d max_ = limits.max();
            //std::cout << "before rectify pose max x y =" << max_.x() << "      " << max_.y() << std::endl;
            Eigen::Vector3d correct_max,correct_min;
            correct_max << max_.x(),max_.y(),0.0;//算出了以前的点在世界坐标系下的值
            Eigen::Matrix3d submap_R,correctsubmap_R;
            Eigen::Vector3d submap_t,correctsubmap_t;
            submap_R = submap_Pose.rotation().toRotationMatrix();//为子图对应的虚拟激光坐标系在全局坐标系下的Ｒ
            submap_t = submap_Pose.translation();//为子图对应的虚拟激光坐标系在全局坐标系下的t
            correctsubmap_t = iter_pose->second.first;//为子图对应的imu坐标系在全局坐标系下的Ｒ
            correctsubmap_R = iter_pose->second.second.toRotationMatrix();

            Eigen::Quaterniond correctsubmap_q;
            correctsubmap_q = iter_pose->second.second;
            double siny_cosp = 2 * (correctsubmap_q.w() * correctsubmap_q.z() + correctsubmap_q.x() * correctsubmap_q.y());
            double cosy_cosp = 1 - 2 * (correctsubmap_q.y() * correctsubmap_q.y()+ correctsubmap_q.z() * correctsubmap_q.z());
            double angle = std::atan2(siny_cosp, cosy_cosp);
            // cout  <<  "theta from atan2:  " << angle << endl;

            Pose2d robot_pose = Pose2d( correctsubmap_t[0], correctsubmap_t[1], angle);

            //* 转换到世界坐标系下 */
            // 激光在世界坐标系 = 机器人在世界坐标系 * 激光在机器人坐标系
            Pose2d laser_pose = robot_pose * T_r_l_;//得到修正后的激光坐标系在世界坐标系下的相对变换

            transform::Rigid3d correctsubmap_l(Eigen::Vector3d(laser_pose.x_, laser_pose.y_, 0.0),
                                               Eigen::Quaterniond(Eigen::AngleAxisd(laser_pose.theta_,Eigen::Vector3d::UnitZ())));////得到修正后的激光坐标系在世界坐标系下的相对变换
            correctsubmap_R = correctsubmap_l.rotation().toRotationMatrix();
            correctsubmap_t = correctsubmap_l.translation();

            correct_max = submap_R.transpose()*correct_max - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
            correct_max = correctsubmap_R * correct_max + correctsubmap_t;//经过修正后的在世界坐标系下的值
            max_ << correct_max.x(),correct_max.y();
            //std::cout << "after rectify pose max x y =" << max_.x() << "     " << max_.y() << std::endl;
            CellLimits cell_limits_ ;
            cell_limits_ = limits.cell_limits();

            Eigen::Vector2d min_;
            Eigen::Vector2f min;
            //对应的是栅格上右下角对应的世界坐标值
            min = limits.GetCellCenter({cell_limits_.num_x_cells,cell_limits_.num_y_cells});
            //std::cout << "before rectify pose min x y =" << min.x() << "     " << min.y() << std::endl;
            correct_min << min.x(),min.y(),0.0;
            correct_min = submap_R.transpose()*correct_min - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
            correct_min = correctsubmap_R * correct_min + correctsubmap_t;//经过修正后的在世界坐标系下的值
            min_ << correct_min.x(),correct_min.y();
            //std::cout << "after rectify pose min x y =" << min_.x() << "     " << min_.y() << std::endl;
            map_scope.extend(max_);
            map_scope.extend(min_);
        }


        Eigen::Array2d Colrow =  (map_scope.max()-map_scope.min())/m_resolution;

        Eigen::Vector2f max, min;
        max << map_scope.max().x(),map_scope.max().y();
        min << map_scope.min().x(),map_scope.min().y();

        //根据子图统计出子图所占据的在世界坐标系下的最大值和最小值，判断这个最大值或最小值是否被m_mapLimit所包含，若不包含则将地图扩大
        //直到128*128，即2.56m*2.56m前，都以2倍扩张的方式四周扩张
        if(m_mapLimit.cell_limits().num_x_cells <= 128){
            while((!m_mapLimit.Contains(m_mapLimit.GetCellIndex(max)))||
            (!m_mapLimit.Contains(m_mapLimit.GetCellIndex(min)))){
                m_map = cv::Mat(m_mapLimit.cell_limits().num_x_cells*2,
                                m_mapLimit.cell_limits().num_y_cells*2,CV_64FC1,cv::Scalar(0.5));
                const float x_offset = float(m_mapLimit.cell_limits().num_x_cells) / 2.0;
                const float y_offset = float(m_mapLimit.cell_limits().num_x_cells) / 2.0;
                Eigen::Array2d max;
                max << m_mapLimit.max().x()+y_offset*m_resolution,
                m_mapLimit.max().y()+x_offset*m_resolution;
                std::cout << "max x,y=------------------" << max.x() << "     " << max.y() << std::endl;
                CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells*2,
                                        m_mapLimit.cell_limits().num_y_cells*2) ;
                MapLimits mapLimits(m_resolution,max,-max,mapCell_limits_);//YH230704
                map_mute.lock();
                m_mapLimit = mapLimits;
                map_mute.unlock();
                map_finish = false;
            }
        }

        while((!m_mapLimit.Contains(m_mapLimit.GetCellIndex(max)))||
        (!m_mapLimit.Contains(m_mapLimit.GetCellIndex(min)))){
                Eigen::Array2d max;
                const float offset = float(increase_pixel) / 2.0;
                max << m_mapLimit.max().x()+offset*m_resolution,
                m_mapLimit.max().y()+offset*m_resolution;
                std::cout << "max x,y=------------------" << max.x() << "     " << max.y() << std::endl;
                CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells+increase_pixel,
                                        m_mapLimit.cell_limits().num_y_cells+increase_pixel) ;
                MapLimits mapLimits(m_resolution,max,-max,mapCell_limits_);//YH230704新增min
                map_mute.lock();
                m_mapLimit = mapLimits;
                map_mute.unlock();
                map_finish = false;
        }
        m_map = cv::Mat(m_mapLimit.cell_limits().num_x_cells,m_mapLimit.cell_limits().num_y_cells,CV_64FC1,cv::Scalar(0.5));
        m_map.setTo(cv::Scalar(0.5));
        
        //std::cout << m_map << std::endl;
        for(auto submap:submaps_){
            Eigen::Array2i offset;
            CellLimits cell_limits;

            auto grid = submap.second->grid();
            //pose_graph
            double key = submap.first;

            auto iter_pose = rectify_path.find(key);
            if(iter_pose==rectify_path.end()){
                std::cout << "can't find correct pose in rectify_path" << std::endl;
                throw;
            }
            transform::Rigid3d submap_Pose = submap.second->local_pose();
            Eigen::Matrix3d submap_R,correctsubmap_R;
            Eigen::Vector3d submap_t,correctsubmap_t;
            submap_R = submap_Pose.rotation().toRotationMatrix();//得到的是子图的坐标
            submap_t = submap_Pose.translation();
            correctsubmap_t = iter_pose->second.first;//得到子图经过回环修正后的坐标
            correctsubmap_R = iter_pose->second.second.toRotationMatrix();

            Eigen::Quaterniond correctsubmap_q;
            correctsubmap_q = iter_pose->second.second;
            double siny_cosp = 2 * (correctsubmap_q.w() * correctsubmap_q.z() + correctsubmap_q.x() * correctsubmap_q.y());
            double cosy_cosp = 1 - 2 * (correctsubmap_q.y() * correctsubmap_q.y()+ correctsubmap_q.z() * correctsubmap_q.z());
            double angle = std::atan2(siny_cosp, cosy_cosp);
            // cout  <<  "theta from atan2:  "  <<  angle  <<  endl;

            Pose2d robot_pose = Pose2d( correctsubmap_t[0], correctsubmap_t[1], angle);

            //* 转换到世界坐标系下 */
            Pose2d laser_pose = robot_pose * T_r_l_;//得到修正后的激光坐标系在世界坐标系下的相对变换

            transform::Rigid3d correctsubmap_l(Eigen::Vector3d(laser_pose.x_, laser_pose.y_, 0.0),
                                               Eigen::Quaterniond(Eigen::AngleAxisd(laser_pose.theta_,Eigen::Vector3d::UnitZ())));////得到修正后的激光坐标系在世界坐标系下的相对变换
            correctsubmap_R = correctsubmap_l.rotation().toRotationMatrix();
            correctsubmap_t = correctsubmap_l.translation();

            MapLimits limit = grid->limits();

            //计算地图有效区域
            grid->ComputeCroppedLimits(&offset, &cell_limits);//offset x对应的是行,cell_limits.num_x_cells也是对应的行

            //pose_graph
            Eigen::Array2i newCell_index;
            Eigen::Vector2f point_word;
            Eigen::Vector2f correct_pointWorld2f;
            Eigen::Vector3d correct_pointWorld;
            // 遍历地图, 将栅格数据存入cells，xy_index的x先加//xy_index对应的也是行列
            for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
                if (!grid->IsKnown(xy_index + offset)) {
                    continue;
                }
                float delta =grid->GetProbability(xy_index + offset);

                point_word = limit.GetCellCenter(xy_index + offset); //得到子地图下的栅格坐标在未修正的全局地图上的坐标
                correct_pointWorld << point_word.x(),point_word.y(),0;//算出了以前的点在世界坐标系下的值
                correct_pointWorld = submap_R.transpose()*correct_pointWorld - submap_R.transpose()*submap_t;//点在未修正的submap坐标系下的值
                //激光在真实世界坐标系下
                correct_pointWorld = correctsubmap_R * correct_pointWorld + correctsubmap_t;//经过修正后的在世界坐标系下的值
                correct_pointWorld2f << correct_pointWorld[0],correct_pointWorld[1];
                newCell_index = m_mapLimit.GetCellIndex(correct_pointWorld2f);//得到修正后的世界坐标系下的值在全局坐标系下的珊格坐标

                if(!m_mapLimit.Contains(m_mapLimit.GetCellIndex(correct_pointWorld2f))){
                    std::cout << "point beyond the scope of map" << std::endl;
                    throw ;
                }
                m_map.at<double>(newCell_index.x(), newCell_index.y())=delta;//第一个参数是行第二个是列
            }
        }
        map_finish = true;
    }
   
   //YH230701
    void ActiveSubmaps2D::buildLargeMapPixelV2(){
        Eigen::AlignedBox2d map_scope;
        Eigen::AlignedBox2d submap_scope;

        for(auto submap:submaps_){
            MapLimits limits = submap.second->grid()->limits();

            Eigen::Vector2d max_ = limits.max();
            CellLimits cell_limits_ ;
            cell_limits_ = limits.cell_limits();
            Eigen::Vector2d min_;
            Eigen::Vector2f min;
            //对应的是珊格上右下角对应的世界坐标值
            min = limits.GetCellCenter({cell_limits_.num_x_cells,cell_limits_.num_y_cells});
            min_ << min.x(),min.y();

            map_scope.extend(max_);
            map_scope.extend(min_);
        }


        Eigen::Array2d Colrow =  (map_scope.max()-map_scope.min())/m_resolution;

        Eigen::Vector2f max, min;
        max << map_scope.max().x(),map_scope.max().y();
        min << map_scope.min().x(),map_scope.min().y();

        //根据子图统计出子图所占据的在世界坐标系下的最大值和最小值，判断这个最大值或最小值是否被m_mapLimit所包含，若不包含则将地图扩大
        //直到128*128，即2.56m*2.56m前，都以2倍扩张的方式四周扩张

        if(m_mapLimit.cell_limits().num_x_cells <= 128){
            while((!m_mapLimit.Contains(m_mapLimit.GetCellIndex(max)))||
            (!m_mapLimit.Contains(m_mapLimit.GetCellIndex(min)))){
                m_map = cv::Mat(m_mapLimit.cell_limits().num_x_cells*2,
                                m_mapLimit.cell_limits().num_y_cells*2,CV_64FC1,cv::Scalar(0.5));
                const float x_offset = float(m_mapLimit.cell_limits().num_x_cells) / 2.0;
                const float y_offset = float(m_mapLimit.cell_limits().num_x_cells) / 2.0;
                Eigen::Array2d max, min;
                max << m_mapLimit.max().x()+y_offset*m_resolution,
                m_mapLimit.max().y()+x_offset*m_resolution;
                min << -max.x(),-max.y();
                std::cout << "max x,y=------------------" << max.x() << "     " << max.y() << std::endl;
                CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells*2,
                                        m_mapLimit.cell_limits().num_y_cells*2) ;
                MapLimits mapLimits(m_resolution,max,min,mapCell_limits_);
                map_mute.lock();
                m_mapLimit = mapLimits;
                cell_start  <<  (m_mapLimit.cell_limits().num_x_cells/2),(m_mapLimit.cell_limits().num_y_cells/2);//YH230704记录中心点
                map_mute.unlock();
                map_finish = false;
            }
        }
        
        //到这里地图num_x_cells扩张到128*128，左上角max_为64*0.05m，右下角min_为-64*0.05m//YH230704
        Eigen::Vector2i origin_save;//YH230704存储起点用的临时变量                    
        Eigen::Array2d _max , _min;//YH230704存放边界点的临时变量
        
        while((!m_mapLimit.Contains(m_mapLimit.GetCellIndex(max)))||
        (!m_mapLimit.Contains(m_mapLimit.GetCellIndex(min)))){
                //这里超限情况有4种：
                //1、左上角横向超限，即max.x()>max_
                while(max.x() >= m_mapLimit.max().x()){
                    _max << m_mapLimit.max().x()+increase_pixel*m_resolution, m_mapLimit.max().y();
                    _min << m_mapLimit.min().x(),m_mapLimit.min().y();
                    std::cout << "SINGLE MODE left plus --------max x,y = " << _max.x() << " " << _max.y() << std::endl;
                    std::cout << "SINGLE MODE left plus --------min x,y = " << _min.x() << " " << _min.y() << std::endl;
                    CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells+increase_pixel, m_mapLimit.cell_limits().num_y_cells) ;
                    MapLimits mapLimits(m_resolution,_max,_min,mapCell_limits_);
                    origin_save  <<  cell_start.x()+increase_pixel, cell_start.y();
                    map_mute.lock();
                    m_mapLimit = mapLimits;
                    cell_start  <<  origin_save.x(),origin_save.y();
                    map_mute.unlock();
                    map_finish = false;
                }
                //2、左上角纵向超限；
                while(max.y() >= m_mapLimit.max().y()){
                    _max << m_mapLimit.max().x(), m_mapLimit.max().y()+increase_pixel*m_resolution;
                    _min << m_mapLimit.min().x(),m_mapLimit.min().y();
                    std::cout << "SINGLE MODE up plus --------max x,y = " << _max.x() << " " << _max.y() << std::endl;
                    std::cout << "SINGLE MODE up plus --------min x,y = " << _min.x() << " " << _min.y() << std::endl;
                    CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells, m_mapLimit.cell_limits().num_y_cells+increase_pixel) ;
                    MapLimits mapLimits(m_resolution,_max,_min,mapCell_limits_);
                    origin_save  <<  cell_start.x(), cell_start.y()+increase_pixel;
                    map_mute.lock();
                    m_mapLimit = mapLimits;
                    cell_start  <<  origin_save.x(),origin_save.y();
                    map_mute.unlock();
                    map_finish = false;
                }
                //3、右下角横向超限，即min.x()<min_；
                while(min.x() <= m_mapLimit.min().x()){
                    _max  << m_mapLimit.max().x(), m_mapLimit.max().y();
                    _min  << m_mapLimit.min().x()-increase_pixel*m_resolution, m_mapLimit.min().y();
                    std::cout << "SINGLE MODE right plus --------max x,y = " << _max.x() << " " << _max.y() << std::endl;
                    std::cout << "SINGLE MODE right plus --------min x,y = " << _min.x() << " " << _min.y() << std::endl;
                    CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells+increase_pixel, m_mapLimit.cell_limits().num_y_cells) ;
                    MapLimits mapLimits(m_resolution,_max,_min,mapCell_limits_);
                    map_mute.lock();
                    m_mapLimit = mapLimits;
                    map_mute.unlock();
                    map_finish = false;
                }
                //4、右下角纵向超限
                while(min.y() <= m_mapLimit.min().y()){
                    _max  << m_mapLimit.max().x(), m_mapLimit.max().y();
                    _min  << m_mapLimit.min().x(), m_mapLimit.min().y()-increase_pixel*m_resolution;
                    std::cout << "SINGLE MODE down plus --------max x,y = " << _max.x() << " " << _max.y() << std::endl;
                    std::cout << "SINGLE MODE down plus --------min x,y = " << _min.x() << " " << _min.y() << std::endl;
                    CellLimits mapCell_limits_(m_mapLimit.cell_limits().num_x_cells, m_mapLimit.cell_limits().num_y_cells+increase_pixel) ;
                    MapLimits mapLimits(m_resolution,_max,_min,mapCell_limits_);
                    map_mute.lock();
                    m_mapLimit = mapLimits;
                    map_mute.unlock();
                    map_finish = false;
                }
                std::cerr << "limit loop ing ....." << std::endl;
                map_finish = false;
        }

        std::cout << "SINGLE MODE m_map creating......" << std::endl;
        m_map = cv::Mat(m_mapLimit.cell_limits().num_x_cells,m_mapLimit.cell_limits().num_y_cells,CV_64FC1,cv::Scalar(0.5));
        m_map.setTo(cv::Scalar(0.5));
        
        //std::cout << m_map << std::endl;
        for(auto submap:submaps_){
            Eigen::Array2i offset;
            CellLimits cell_limits;

            auto grid = submap.second->grid();

            MapLimits limit = grid->limits();

            //计算地图有效区域
            grid->ComputeCroppedLimits(&offset, &cell_limits);//offset x对应的是行,cell_limits.num_x_cells也是对应的行

            // 遍历地图, 将栅格数据存入cells，xy_index的x先加//xy_index对应的也是行列
            for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
                if (!grid->IsKnown(xy_index + offset)) {
                    continue;
                }
                float delta =grid->GetProbability(xy_index + offset);


                //得到子地图下的栅格坐标在全局地图珊格上的坐标
                auto max_xy = m_mapLimit.GetCellIndex(limit.GetCellCenter(xy_index + offset));
                if(!m_mapLimit.Contains(m_mapLimit.GetCellIndex(limit.GetCellCenter(xy_index + offset)))){
                    std::cout << "point beyond the scope of map" << std::endl;
                    throw ;
                }
                m_map.at<double>(max_xy.x(), max_xy.y())=delta;//第一个参数是行第二个是列
            }
        }
        map_finish = true;

    }
   


    void ActiveSubmaps2D::saveMap()
    {
        static int count = 0;
        std::cout  <<  std::endl  <<  "[Map Debug] saveMap thread start!" <<  std::endl;
        std::cout  <<  "[Map Debug] Map save path is " <<  MAP_SAVE_PATH  <<  std::endl;

        // std::unique_lock<std::mutex> locker(map_mute);//YH230704
        // std::condition_variable con_savemap;//YH230704

        while(1){
            char c = getchar();
            if (c == 's') //键盘输入s手动保存一个地图
            {
                while(!map_finish){
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
                // con_savemap.wait(locker);
                m_map_clone = m_map.clone();
                cv::Mat savegrid(m_map_clone.rows,m_map_clone.cols,CV_8UC1,cv::Scalar(0));
                for(int i = 0;i<m_map_clone.rows;i++){
                    for(int j=0;j<m_map_clone.cols;j++){
                        if(0<=int(m_map_clone.at<double>(i,j)*255)<=255){
                            savegrid.at<uchar>(i,j) = int(m_map_clone.at<double>(i,j)*255);
                        }
                        else{
                            std::cout  <<  "probability value error" << std::endl;
                            throw;
                        }
                    }
                }
                Eigen::Vector2f origin;
                origin << 0,0;
                auto origin_row_col = m_mapLimit.GetCellIndex(origin);
 
                if ((count % 100 == 0) && (SAVE_MAP == 1))
                {  
                    // cv::imwrite(MAP_SAVE_PATH+"/map/map"+std::to_string(count)+".pgm",savegrid);//分阶段大地图       
                    cv::imwrite(MAP_SAVE_PATH+"map/map.pgm",savegrid);//单张大地图              
                    cv::FileStorage fs_write(MAP_SAVE_PATH+"map/map.yaml", cv::FileStorage::WRITE);
                    fs_write << "resolution" << m_resolution;
                    fs_write << "origin_row" << origin_row_col.x();
                    fs_write << "origin_col" << origin_row_col.y();
                    fs_write << "YH Mode origin_row " << cell_start.y();
                    fs_write << "YH Mode origin_col " << cell_start.x();
                    fs_write.release();
                    std::cout << "write map finished" << std::endl;
                    count = 0;
                }
                count ++;
                map_intfc = savegrid.clone();//YH230619
            }
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    void ActiveSubmaps2D::insertScan(sensor_msgs::msg::LaserScan::ConstPtr Scan,Eigen::Matrix<double, 7, 1> pose_scan,double header){
        //
        RangeData range_data;
        // convertLaserScanToRangeData(range_data,Scan,pose_scan);
        convertLaserScanToRangeDataV2(range_data,Scan,pose_scan);
        std::vector<std::pair<double,std::shared_ptr<Submap>>> insertion_submaps = InsertRangeData(range_data,header);
        if(int(insertion_submaps.size())==16){
            cv::Mat showSubmap;
            insertion_submaps.back().second->grid()->ProbabilityGridPicture(showSubmap);
            // cv::imshow("showSubmap",showSubmap);
            std::string s;
            int i=0;
            for(auto it:insertion_submaps){
                auto Grid = it.second->grid();

                cv::Mat showSubmap;
                i++;
                s+= std::to_string(i);

                Grid->ProbabilityGridPicture(showSubmap);
                cv::namedWindow("showSubmap"+s,cv::WINDOW_AUTOSIZE);
                cv::imshow("showSubmap"+s,showSubmap);
                cv::waitKey(100);
            }
            // cv::waitKey();
        }
}

    void ActiveSubmaps2D::publishMapToRos(const std::string& frame_id){

        nav_msgs::msg::OccupancyGrid occ_grid;

        occ_grid.header.frame_id = frame_id;
        // occ_grid.header.stamp = rclcpp::Clock.now();
        occ_grid.info.width = m_mapLimit.cell_limits().num_x_cells;
        occ_grid.info.height = m_mapLimit.cell_limits().num_y_cells;
        occ_grid.info.resolution = m_resolution;

        Eigen::Array2i origin;
        origin << m_mapLimit.cell_limits().num_x_cells-1,m_mapLimit.cell_limits().num_y_cells-1;
        Eigen::Array2f originrowcol = m_mapLimit.GetCellCenter(origin);
        //为图像右小角对应的物理坐标
        occ_grid.info.origin.position.x = originrowcol.x();
        occ_grid.info.origin.position.y = originrowcol.y();
//         occ_grid.info.origin.position.x = 0.0;
//         occ_grid.info.origin.position.y = 0.0;
//        occ_grid.info.origin.position.z = 0.0;

        cv::Mat map_clone = m_map.clone();
        cv::flip(map_clone, map_clone, 0);
        cv::flip(map_clone, map_clone, 1);
        for(int i=0;i<map_clone.rows;i++){
            for(int j=0;j<map_clone.cols;j++){
                double value = map_clone.at<double>(i, j);
                if(value == 0.5){
                    occ_grid.data.push_back( -1);
                }
                else if(value < 0.41){
                    occ_grid.data.push_back(value*10);
                }
                else{
                    occ_grid.data.push_back( value * 100);
                }
            }
        }
        cv::imshow("map_clone",map_clone);
        cv::waitKey(1);

//        const int N = m_map.rows * m_map.cols;
//        for(size_t i= 0; i < N; i ++)
//        {
//            double value = m_map.data[i];
//            if(value == 0.5)
//                occ_grid.data.push_back( -1);
//            else
//                occ_grid.data.push_back( value * 100);
//        }
//        std::cout << m_map.data << std::endl;
        mapPublisher->publish(occ_grid);
    }

    void ActiveSubmaps2D::registerPub(rclcpp::Node::SharedPtr node){
        mapPublisher  = node->create_publisher<nav_msgs::msg::OccupancyGrid>("upgradeGrid_map", 1);
    }

}

