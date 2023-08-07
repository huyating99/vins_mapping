//
// Created by lao-tang on 2023/3/27.
//
#include "probability_value.h"
#include "submap_limit.h"
#include "value_conversion_tables.h"
#include "probabilityGrid.h"

namespace mapping{
    constexpr int kValueCount = 32768;

    /**
 * @brief 构造函数
 *
 * @param[in] limits 地图坐标信息
 * @param[in] min_correspondence_cost 最小correspondence_cost 0.1
 * @param[in] max_correspondence_cost 最大correspondence_cost 0.9
 * @param[in] conversion_tables 传入的转换表指针
 */
    ProbabilityGrid::ProbabilityGrid(const MapLimits& limits, float min_correspondence_cost,
                   float max_correspondence_cost,
                   ValueConversionTables* conversion_tables)
            : limits_(limits),
              correspondence_cost_cells_(
                      limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
                      kUnknownCorrespondenceValue),  // 0
              min_correspondence_cost_(min_correspondence_cost),  // 0.1
              max_correspondence_cost_(max_correspondence_cost),  // 0.9
            // 新建转换表
              value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
                      max_correspondence_cost, min_correspondence_cost,
                      max_correspondence_cost)), conversion_tables_(conversion_tables) {

        ICHECK_LT(min_correspondence_cost_,max_correspondence_cost_);
    }

    // Finishes the update sequence.
// 插入雷达数据结束
    void ProbabilityGrid::FinishUpdate() {
        while (!update_indices_.empty()) {
            ICHECK_GE(correspondence_cost_cells_[update_indices_.back()],
                      kUpdateMarker);
            // 更新的时候加上了kUpdateMarker, 在这里减去
            correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
            update_indices_.pop_back();
        }
    }

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
// 根据known_cells_box_更新limits
    void ProbabilityGrid::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                      CellLimits* const limits) const {
        if (known_cells_box_.isEmpty()) {
            *offset = Eigen::Array2i::Zero();
            *limits = CellLimits(1, 1);
            return;
        }
        *offset = known_cells_box_.min().array();
        *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                             known_cells_box_.sizes().y() + 1);
    }

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
// 根据坐标决定是否对地图进行扩大
    void ProbabilityGrid::GrowLimits(const Eigen::Vector2f& point) {
        GrowLimits(point, {mutable_correspondence_cost_cells()},{kUnknownCorrespondenceValue});
    }

// 根据坐标决定是否对地图进行扩大
    void ProbabilityGrid::GrowLimits(const Eigen::Vector2f& point,
                            const std::vector<std::vector<uint16>*>& grids,
                            const std::vector<uint16>& grids_unknown_cell_values) {
        ICHECK(update_indices_.empty());
        // 判断该点是否在地图坐标系内
        while (!limits_.Contains(limits_.GetCellIndex(point))) {
            const int x_offset = limits_.cell_limits().num_x_cells / 2;
            const int y_offset = limits_.cell_limits().num_y_cells / 2;
            // 将xy扩大至2倍, 中心点不变, 向四周扩大
            const MapLimits new_limits(
                    limits_.resolution(),
                    limits_.max() +
                    limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
                    limits_.min() -
                    limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
                    CellLimits(2 * limits_.cell_limits().num_x_cells,
                               2 * limits_.cell_limits().num_y_cells));//YH230704
            const int stride = new_limits.cell_limits().num_x_cells;
            // 老坐标系的原点在新坐标系下的一维像素坐标
            const int offset = x_offset + stride * y_offset;
            const int new_size = new_limits.cell_limits().num_x_cells *
                                 new_limits.cell_limits().num_y_cells;

            // grids.size()为1
            for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
                std::vector<uint16> new_cells(new_size,
                                              grids_unknown_cell_values[grid_index]);
                // 将老地图的栅格值复制到新地图上
                for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
                    for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
                        new_cells[offset + j + i * stride] =
                                (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
                    }
                }
                // 将新地图替换老地图, 拷贝
                *grids[grid_index] = new_cells;
            } // end for
            // 更新地图尺寸
            limits_ = new_limits;
            if (!known_cells_box_.isEmpty()) {
                // 将known_cells_box_的x与y进行平移到老地图的范围上
                known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
            }
        }
    }

    // 将 索引 处单元格的概率设置为给定的概率, 仅当单元格之前处于未知状态时才允许
    void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,const float probability) {
        // 获取对应栅格的引用
        uint16& cell =(*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
        ICHECK_EQ(cell, kUnknownProbabilityValue);
        // 为栅格赋值 value
        cell =CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
        // 更新bounding_box
        mutable_known_cells_box()->extend(cell_index.matrix());
    }

    // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
// 如果单元格尚未更新,则将调用 ComputeLookupTableToApplyOdds() 时指定的 'odds' 应用于单元格在 'cell_index' 处的概率
// 在调用 FinishUpdate() 之前，将忽略同一单元格的多次更新。如果单元格已更新，则返回 true
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// 如果这是对指定单元格第一次调用 ApplyOdds(),则其值将设置为与 'odds' 对应的概率

// 使用查找表对指定栅格进行栅格值的更新
    bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,const std::vector<uint16>& table)
    {
        ICHECK_EQ(table.size(), kUpdateMarker);
        const int flat_index = ToFlatIndex(cell_index);
        // 获取对应栅格的指针
        uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
        // 对处于更新状态的栅格, 不再进行更新了
        if (*cell >= kUpdateMarker) {
            return false;
        }
        // 标记这个索引的栅格已经被更新过
        mutable_update_indices()->push_back(flat_index);
        // 更新栅格值
        *cell = table[*cell];
        ICHECK_GE(*cell, kUpdateMarker);
        // 更新bounding_box
        mutable_known_cells_box()->extend(cell_index.matrix());
        return true;
    }

    // Returns the probability of the cell with 'cell_index'.
// 获取 索引 处单元格的占用概率
    float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
        if (!limits().Contains(cell_index)) {
            std::cout<<"cell_index is not included by grid !!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            return kMinProbability;}
        return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
                correspondence_cost_cells()[ToFlatIndex(cell_index)]));
    }

    // 根据bounding_box对栅格地图进行裁剪到正好包含点云
    std::unique_ptr<ProbabilityGrid> ProbabilityGrid::ComputeCroppedGrid() const {
        Eigen::Array2i offset;
        CellLimits cell_limits;
        // 根据bounding_box对栅格地图进行裁剪
        ComputeCroppedLimits(&offset, &cell_limits);
        const double resolution = limits().resolution();
        // 重新计算最大值坐标
        const Eigen::Vector2d max =
                limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
        // 重新定义概率栅格地图的大小
        std::unique_ptr<ProbabilityGrid> cropped_grid =
                std::make_unique<ProbabilityGrid>(
                        MapLimits(resolution, max, -max, cell_limits), kMinCorrespondenceCost,kMaxCorrespondenceCost,conversion_tables_);//YH230704
        // 给新栅格地图赋值
        for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
            if (!IsKnown(xy_index + offset)) continue;
            cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
        }

        // 返回新地图的指针
        return std::unique_ptr<ProbabilityGrid>(cropped_grid.release());
    }

    void ProbabilityGrid::ProbabilityGridPicture(cv::Mat &show) const{
        Eigen::Array2i offset;
        CellLimits cell_limits;

        //计算地图有效区域
        ComputeCroppedLimits(&offset, &cell_limits);//offset x对应的是行,cell_limits.num_x_cells也是对应的行


//        offset =  Eigen::Array2i::Zero();
//        cell_limits = limits_.cell_limits();

        show=cv::Mat(cell_limits.num_x_cells,cell_limits.num_y_cells,CV_64FC1,cv::Scalar(0.5));
        // 遍历地图, 将栅格数据存入cells，xy_index的x先加//xy_index对应的也是行列
        for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
            if (!IsKnown(xy_index + offset)) {
                continue;
            }
            float delta =GetProbability(xy_index + offset);
            show.at<double>(xy_index.x(), xy_index.y())=delta;//第一个参数是行第二个是列
        }
//        Eigen::Array2i xy;
//        for(int i=0;i<limits_.cell_limits().num_x_cells;i++){
//            for(int j=0;j<limits_.cell_limits().num_y_cells;j++){
//                xy<<i,j;
//                if(j==(limits_.cell_limits().num_y_cells-1))
//                    std::cout<<","<<GetProbability(xy)<<std::endl;
//                    else
//                    std::cout<<","<<GetProbability(xy);
//            }
//        }
    }
}
