//
// Created by lao-tang on 2023/3/27.
//

#ifndef BETA_PROBABILITYGRID_H
#define BETA_PROBABILITYGRID_H

#include "submap_limit.h"
#include "value_conversion_tables.h"
#include <opencv2/opencv.hpp>

namespace mapping{
    class ProbabilityGrid {
    public:
        ProbabilityGrid(const MapLimits& limits, float min_correspondence_cost,
                        float max_correspondence_cost,
                        ValueConversionTables* conversion_tables);

        // Returns the limits of this Grid2D.
        const MapLimits& limits() const { return limits_; }

        // Converts a 'cell_index' into an index into 'cells_'.
        // 二维像素坐标转为一维索引坐标
        int ToFlatIndex(const Eigen::Array2i& cell_index) const {
            ICHECK(limits_.Contains(cell_index)) ;
            return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
        }

        // Returns the correspondence cost of the cell with 'cell_index'.
        float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
            if (!limits().Contains(cell_index)) return max_correspondence_cost_;
            return (*value_to_correspondence_cost_table_)
            [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
        }

        // 返回不可以修改的栅格地图数组的引用
        const std::vector<uint16>& correspondence_cost_cells() const {return correspondence_cost_cells_;}

        const std::vector<int>& update_indices() const { return update_indices_; }

        const Eigen::AlignedBox2i& known_cells_box() const {return known_cells_box_;}

        // 返回可以修改的栅格地图数组的指针
        std::vector<uint16>* mutable_correspondence_cost_cells() {return &correspondence_cost_cells_;}

        std::vector<int>* mutable_update_indices() { return &update_indices_; }
        Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

        void FinishUpdate();

        // Returns the minimum possible correspondence cost.
        float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

        // Returns the maximum possible correspondence cost.
        float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

        // Returns true if the probability at the specified index is known.
        bool IsKnown(const Eigen::Array2i& cell_index) const {
            return limits_.Contains(cell_index) &&correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
                   kUnknownCorrespondenceValue;
        }

        void ComputeCroppedLimits(Eigen::Array2i* const offset,CellLimits* const limits) const;
        void GrowLimits(const Eigen::Vector2f& point);
        void GrowLimits(const Eigen::Vector2f& point,const std::vector<std::vector<uint16>*>& grids,
                                         const std::vector<uint16>& grids_unknown_cell_values);

        //they include grid map in cartographer before this,after all include probability in cartographer
        void SetProbability(const Eigen::Array2i& cell_index,const float probability);
        bool ApplyLookupTable(const Eigen::Array2i& cell_index,const std::vector<uint16>& table);
        float GetProbability(const Eigen::Array2i& cell_index) const;
        std::unique_ptr<ProbabilityGrid> ComputeCroppedGrid() const;

        void ProbabilityGridPicture(cv::Mat &show) const;

    private:
        MapLimits limits_;  // 地图大小边界, 包括x和y最大值, 分辨率, x和y方向栅格数

        // 地图栅格值, 存储的是free的概率转成uint16后的[0, 32767]范围内的值, 0代表未知
        std::vector<uint16> correspondence_cost_cells_;
        float min_correspondence_cost_;
        float max_correspondence_cost_;
        std::vector<int> update_indices_;               // 记录已经更新过的索引

        // Bounding box of known cells to efficiently compute cropping limits.
        Eigen::AlignedBox2i known_cells_box_;           // 栅格的bounding box, 存的是像素坐标,为行列
        // 将[0, 1~32767] 映射到 [0.9, 0.1~0.9] 的转换表
        const std::vector<float>* value_to_correspondence_cost_table_;
        ValueConversionTables* conversion_tables_;
    };
}

#endif //BETA_PROBABILITYGRID_H
