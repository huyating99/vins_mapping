//
// Created by lao-tang on 2023/3/27.
//


#ifndef BETA_PROBABILITYGRIDRANGEDATEINSETER_H
#define BETA_PROBABILITYGRIDRANGEDATEINSETER_H

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./probabilityGrid.h"
// #include "../common/sensorMsg.h"
#include "point_cloud.h"
// #include "../utility/parameters.h"
#include "math_map.h"

namespace mapping{

    /**
 * @brief 中存储所有雷达点云的数据结构
 *
 * @param origin  点云的原点在local坐标系下的坐标
 * @param returns 所有雷达数据点在local坐标系下的坐标, 记为returns, 也就是hit
 * @param misses  是在光线方向上未检测到返回的点(nan, inf等等)或超过最大配置距离的点
 */
    struct RangeData {
        Eigen::Vector3f origin;
        sensor::PointCloud returns;//全局坐标系下的点
        sensor::PointCloud misses;//全局坐标系下的点
    };

    class ProbabilityGridRangeDateInseter{
    public:
        ProbabilityGridRangeDateInseter();
        void Insert(const RangeData& range_data,ProbabilityGrid* grid) const;

    private:
        const std::vector<uint16> hit_table_;
        const std::vector<uint16> miss_table_;

    };

}


#endif //BETA_PROBABILITYGRIDRANGEDATEINSETER_H
