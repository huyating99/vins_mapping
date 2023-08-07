//
// Created by lao-tang on 2023/3/27.
//

#include "submap.h"
#include "probabilityGrid.h"

namespace mapping{

    /**
* @brief 构造函数
*
* @param[in] origin Submap2D的原点,保存在Submap类里
* @param[in] grid 地图数据的指针
* @param[in] conversion_tables 地图数据的转换表
*/
    Submap::Submap(const Eigen::Vector3f& origin, std::unique_ptr<ProbabilityGrid> grid,
                       ValueConversionTables* conversion_tables)
            : local_pose_(transform::Rigid3d(Eigen::Vector3d(origin.x(), origin.y(), 0.),
                          Eigen::Quaterniond(Eigen::AngleAxisd(origin.z(),Eigen::Vector3d::UnitZ())))),
              conversion_tables_(conversion_tables) {
        grid_ = std::move(grid);
    }

    // 将雷达数据写到栅格地图中
    void Submap::InsertRangeData(
            const RangeData& range_data,
            const ProbabilityGridRangeDateInseter* range_data_inserter) {
        if(grid_==NULL) throw;
        ICHECK(!insertion_finished());
        // 将雷达数据写到栅格地图中
        range_data_inserter->Insert(range_data, grid_.get());
        // 插入到地图中的雷达数据的个数加1
        set_num_range_data(num_range_data() + 1);

//        cv::Mat showSubmap;
//        grid_->ProbabilityGridPicture(showSubmap);
//        cv::imshow("showSubmap",showSubmap);
    }

// 将子图标记为完成状态
    void Submap::Finish() {
        if(grid_==NULL) throw;
        ICHECK(!insertion_finished());
        grid_ = grid_->ComputeCroppedGrid();
        // 将子图标记为完成状态
        set_insertion_finished(true);
    }

}


