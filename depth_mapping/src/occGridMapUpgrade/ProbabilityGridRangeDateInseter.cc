//
// Created by lao-tang on 2023/3/27.
//

#include"probability_value.h"
#include "ProbabilityGridRangeDateInseter.h"
#include "ray_to_pixel_mask.h"
#include "map.h"

namespace mapping {
    namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
        constexpr int kSubpixelScale = 1000;

// 根据点云的bounding box, 看是否需要对地图进行扩张
        void GrowAsNeeded(const RangeData& range_data,ProbabilityGrid* const probability_grid) {
            // 找到点云的bounding_box
            Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
            // Padding around bounding box to avoid numerical issues at cell boundaries.
            constexpr float kPadding = 1e-6f;
            for (const sensor::RangefinderPoint& hit : range_data.returns) {
                bounding_box.extend(hit.position.head<2>());
            }
            // 是否对地图进行扩张
            probability_grid->GrowLimits(bounding_box.min() -
                                         kPadding * Eigen::Vector2f::Ones());
            probability_grid->GrowLimits(bounding_box.max() +
                                         kPadding * Eigen::Vector2f::Ones());
        }

/**
 * @brief 根据雷达点对栅格地图进行更新
 *
 * @param[in] range_data
 * @param[in] hit_table 更新占用栅格时的查找表
 * @param[in] miss_table 更新空闲栅格时的查找表
 * @param[in] insert_free_space
 * @param[in] probability_grid 栅格地图
 */
        void CastRays(const RangeData& range_data,
                      const std::vector<uint16>& hit_table,
                      const std::vector<uint16>& miss_table,
                      const bool insert_free_space, ProbabilityGrid* probability_grid) {
            // 根据雷达数据调整地图范围
            GrowAsNeeded(range_data, probability_grid);

            const MapLimits& limits = probability_grid->limits();
            const double superscaled_resolution = limits.resolution() / kSubpixelScale;
            const MapLimits superscaled_limits(
                    superscaled_resolution, limits.max(), limits.min(),
                    CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                               limits.cell_limits().num_y_cells * kSubpixelScale));//YH230704
            // 雷达原点在地图中的像素坐标, 作为画线的起始坐标
            const Eigen::Array2i begin =superscaled_limits.GetCellIndex(range_data.origin.head<2>());
            // Compute and add the end points.
            std::vector<Eigen::Array2i> ends;
            ends.reserve(range_data.returns.size());
            for (const sensor::RangefinderPoint& hit : range_data.returns) {
                // 计算hit点在地图中的像素坐标, 作为画线的终止点坐标
                ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
                // 更新hit点的栅格值
                probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
            }

            // 如果不插入free空间就可以结束了
            if (!insert_free_space) {
                return;
            }

            // Now add the misses.
            for (const Eigen::Array2i& end : ends) {
                std::vector<Eigen::Array2i> ray =RayToPixelMask(begin, end, kSubpixelScale);
                for (const Eigen::Array2i& cell_index : ray) {
                    // 从起点到end点之前, 更新miss点的栅格值
                    probability_grid->ApplyLookupTable(cell_index, miss_table);
                }
            }

            // Finally, compute and add empty rays based on misses in the range data.
            for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
                std::vector<Eigen::Array2i> ray = RayToPixelMask(
                        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
                        kSubpixelScale);
                for (const Eigen::Array2i& cell_index : ray) {
                    // 从起点到misses点之前, 更新miss点的栅格值
                    probability_grid->ApplyLookupTable(cell_index, miss_table);
                }
            }
        }

        void CastRaysV2(const RangeData& range_data,
                      const std::vector<uint16>& hit_table,
                      const std::vector<uint16>& miss_table,
                      const bool insert_free_space, ProbabilityGrid* probability_grid) {
            // 根据雷达数据调整地图范围
            GrowAsNeeded(range_data, probability_grid);

            const MapLimits& limits = probability_grid->limits();
            const double superscaled_resolution = limits.resolution() / kSubpixelScale;
            const MapLimits superscaled_limits(
                    superscaled_resolution, limits.max(), limits.min(),
                    CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                               limits.cell_limits().num_y_cells * kSubpixelScale));//YH230704
            // 雷达原点在地图中的像素坐标, 作为画线的起始坐标
            const Eigen::Array2i begin =superscaled_limits.GetCellIndex(range_data.origin.head<2>());
            // Compute and add the end points.
            std::vector<Eigen::Array2i> ends;
            ends.reserve(range_data.returns.size());
            for (const sensor::RangefinderPoint& hit : range_data.returns) {
                // 计算hit点在地图中的像素坐标, 作为画线的终止点坐标
                ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
                // 更新hit点的栅格值
                if(hit.status){
                    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);//不更新没hit中的点
                }
            }

            // 如果不插入free空间就可以结束了
            if (!insert_free_space) {
                return;
            }

            // Now add the misses.
            for (const Eigen::Array2i& end : ends) {
                std::vector<Eigen::Array2i> ray =RayToPixelMask(begin, end, kSubpixelScale);
                for (const Eigen::Array2i& cell_index : ray) {
                    // 从起点到end点之前, 更新miss点的栅格值
                    probability_grid->ApplyLookupTable(cell_index, miss_table);
                }
            }

            // Finally, compute and add empty rays based on misses in the range data.
            for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
                std::vector<Eigen::Array2i> ray = RayToPixelMask(
                        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
                        kSubpixelScale);
                for (const Eigen::Array2i& cell_index : ray) {
                    // 从起点到misses点之前, 更新miss点的栅格值
                    probability_grid->ApplyLookupTable(cell_index, miss_table);
                }
            }
        }
    }  // namespace
    


    // 写入器的构造, 新建了2个查找表
    ProbabilityGridRangeDateInseter::ProbabilityGridRangeDateInseter():   // 生成更新占用栅格时的查找表 // param: hit_probability
    hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(static_cast<float>(P_OCC)))),   // 0.55
    // 生成更新空闲栅格时的查找表 // param: miss_probability
    miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(static_cast<float>(P_FREE))))  // 0.49
    {
      std::cout<<"P_OCC P_FREE in ProbabilityGridRangeDateInseter: " <<static_cast<float>(P_OCC)<<"   "<<P_FREE<<std::endl;
    }

    void ProbabilityGridRangeDateInseter::Insert(const RangeData& range_data,ProbabilityGrid* const grid)const {
        if(grid == nullptr) throw ;
        // By not finishing the update after hits are inserted, we give hits priority
        // (i.e. no hits will be ignored because of a miss in the same cell).
        // param: insert_free_space

        // CastRays(range_data, hit_table_, miss_table_, true,grid);
        CastRaysV2(range_data, hit_table_, miss_table_, true, grid);//YH230705
        grid->FinishUpdate();
    }

}


