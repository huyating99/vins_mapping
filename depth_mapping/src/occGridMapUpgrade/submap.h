//
// Created by lao-tang on 2023/3/27.
//

#ifndef BETA_SUBMAP_H
#define BETA_SUBMAP_H

#include <memory>
#include <vector>
#include "rigid_transform.h"
#include "probability_value.h"
#include "ProbabilityGridRangeDateInseter.h"


namespace mapping{
// Converts the given probability to log odds.
// 对论文里的 odds(p)函数 又取了 log
    inline float Logit(float probability) {
        return std::log(probability / (1.f - probability));
    }

    const float kMaxLogOdds = Logit(kMaxProbability);
    const float kMinLogOdds = Logit(kMinProbability);

    // Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
   // kMaxLogOdds] is mapped to [1, 255].
    inline uint8 ProbabilityToLogOddsInteger(const float probability) {
        const int value = RoundToInt((Logit(probability) - kMinLogOdds) *
                                             254.f / (kMaxLogOdds - kMinLogOdds)) +
                          1;
        // std::cout<<"kMaxProbability "<<kMaxProbability<<"kMinProbability   "<<kMinProbability<<
        // "kMaxLogOdds"<<kMaxLogOdds<<"  kMinLogOdds"<<kMinLogOdds<<std::endl;
        ICHECK_LE(1, value);
        ICHECK_GE(255, value);
        return value;
    }

   /* *
  * @brief 独立的子地图, 3个功能
  *
  * 保存在local坐标系下的子图的坐标
  * 记录插入到子图中雷达数据的个数
  * 标记这个子图是否是完成状态
*/
    class Submap {
    public:

        Submap(const Eigen::Vector3f& origin, std::unique_ptr<ProbabilityGrid> grid,ValueConversionTables* conversion_tables);

        virtual ~Submap() {}


        const ProbabilityGrid* grid() const { return grid_.get(); }
        // Pose of this submap in the local map frame.
        // 在local坐标系的子图的坐标
        transform::Rigid3d local_pose() const { return local_pose_; }

        // Number of RangeData inserted.
        // 插入到子图中雷达数据的个数
        int num_range_data() const { return num_range_data_; }
        void set_num_range_data(const int num_range_data) {
            num_range_data_ = num_range_data;
        }

        bool insertion_finished() const { return insertion_finished_; }
        // 将子图标记为完成状态
        void set_insertion_finished(bool insertion_finished) {
            insertion_finished_ = insertion_finished;
        }

        // Insert 'range_data' into this submap using 'range_data_inserter'. The
        // submap must not be finished yet.
        void InsertRangeData(const RangeData& range_data,
                             const ProbabilityGridRangeDateInseter* range_data_inserter);
        void Finish();

    private:
        const transform::Rigid3d local_pose_; // 子图原点在local坐标系下的坐标
        int num_range_data_ = 0;
        bool insertion_finished_ = false;

        std::unique_ptr<ProbabilityGrid> grid_; // 地图栅格数据

        // 转换表, 第[0-32767]位置, 存的是[0.9, 0.1~0.9]的数据
        ValueConversionTables* conversion_tables_;
    };
}


#endif //BETA_SUBMAP_H
