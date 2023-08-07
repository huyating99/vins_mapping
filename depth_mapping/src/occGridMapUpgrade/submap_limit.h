//
// Created by lao-tang on 2022/7/3.
//

#ifndef BETA_SUBMAP_LIMIT_H
#define BETA_SUBMAP_LIMIT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "math_map.h"

namespace mapping{
    // 珊格行数为num_x_cells与珊格列数为num_y_cells
    struct CellLimits {
        CellLimits() = default;
        CellLimits(int init_num_x_cells, int init_num_y_cells)
                : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

        explicit CellLimits(const CellLimits& cell_limits)
                : num_x_cells(cell_limits.num_x_cells),
                  num_y_cells(cell_limits.num_y_cells) {}

        int num_x_cells = 0;
        int num_y_cells = 0;
    };

    // Iterates in row-major order through a range of xy-indices.
    class XYIndexRangeIterator: public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
    public:
        // Constructs a new iterator for the specified range.
        XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                             const Eigen::Array2i& max_xy_index)
                : min_xy_index_(min_xy_index),
                  max_xy_index_(max_xy_index),
                  xy_index_(min_xy_index) {}

        // Constructs a new iterator for everything contained in 'cell_limits'.
        explicit XYIndexRangeIterator(const CellLimits& cell_limits)
                : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                                       Eigen::Array2i(cell_limits.num_x_cells - 1,
                                                      cell_limits.num_y_cells - 1)) {}

        XYIndexRangeIterator& operator++() {
            // This is a necessary evil. Bounds checking is very expensive and needs to
            // be avoided in production. We have unit tests that exercise this check
            // in debug mode.
            mapping::ICHECK(*this != end());
            if (xy_index_.x() < max_xy_index_.x()) {
                ++xy_index_.x();
            } else {
                xy_index_.x() = min_xy_index_.x();
                ++xy_index_.y();
            }
            return *this;
        }

        Eigen::Array2i& operator*() { return xy_index_; }

        bool operator==(const XYIndexRangeIterator& other) const {
            return (xy_index_ == other.xy_index_).all();
        }

        bool operator!=(const XYIndexRangeIterator& other) const {
            return !operator==(other);
        }

        XYIndexRangeIterator begin() {
            return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
        }

        XYIndexRangeIterator end() {
            XYIndexRangeIterator it = begin();
            it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
            return it;
        }

    private:
        Eigen::Array2i min_xy_index_;
        Eigen::Array2i max_xy_index_;
        Eigen::Array2i xy_index_;
    };

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.

/**
 * note: 地图坐标系可视化展示
 * ros的地图坐标系    cartographer的地图坐标系     cartographer地图的像素坐标系
 *
 * ^ y                            ^ x              0------> x
 * |                              |                |
 * |                              |                |
 * 0 ------> x           y <------0                y
 *
 * ros的地图坐标系: 左下角为原点, 向右为x正方向, 向上为y正方向, 角度以x轴正向为0度, 逆时针为正
 * cartographer的地图坐标系: 坐标系右下角为原点, 向上为x正方向, 向左为y正方向
 *             角度正方向以x轴正向为0度, 逆时针为正
 * cartographer地图的像素坐标系: 左上角为原点, 向右为x正方向, 向下为y正方向
 */
    class MapLimits {
    public:
        /**
         * @brief 构造函数
         *
         * @param[in] resolution 地图分辨率
         * @param[in] max 左上角的坐标为地图坐标的最大值
         * @param[in] cell_limits 地图x方向与y方向的格子数？
         */

        //YH230704新增min
        MapLimits(const double resolution, const Eigen::Vector2d& max, const Eigen::Vector2d& min,
                  const CellLimits& cell_limits)
                : resolution_(resolution), max_(max),min_(min), cell_limits_(cell_limits) {
            mapping::ICHECK_GT(resolution_, 0.);
            mapping::ICHECK_GT(cell_limits.num_x_cells, 0.);
            mapping::ICHECK_GT(cell_limits.num_y_cells, 0.);
        }


        // Returns the cell size in meters. All cells are square and the resolution is
        // the length of one side.
        double resolution() const { return resolution_; }

        // Returns the corner of the limits, i.e., all pixels have positions with
        // smaller coordinates.
        // 返回左上角坐标, 左上角坐标为整个坐标系坐标的最大值
        const Eigen::Vector2d& max() const { return max_;}

        const Eigen::Vector2d& min() const { return min_; }//YH230704

        // Returns the limits of the grid in number of cells.
        const CellLimits& cell_limits() const { return cell_limits_; }

        // Returns the index of the cell containing the 'point' which may be outside
        // the map, i.e., negative or too large indices that will return false for
        // Contains().
        // 计算物理坐标点的像素索引
        Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
            // Index values are row major and the top left has Eigen::Array2i::Zero()
            // and contains (centered_max_x, centered_max_y). We need to flip and
            // rotate.
            return Eigen::Array2i(
                   mapping::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
                   mapping::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
        }

        // Returns the center of the cell at 'cell_index'.
        // 根据像素索引算物理坐标
        Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const {
            return {max_.x() - resolution() * (cell_index[1] + 0.5),
                    max_.y() - resolution() * (cell_index[0] + 0.5)};
        }

        // Returns true if the ProbabilityGrid contains 'cell_index'.
        // 判断给定像素索引是否在栅格地图内部
        bool Contains(const Eigen::Array2i& cell_index) const {
            return (Eigen::Array2i(0, 0) <= cell_index).all() &&
                   (cell_index <
                    Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
                           .all();
        }

        

    private:
        double resolution_;
        Eigen::Vector2d max_;    // cartographer地图坐标系左上角为坐标系的坐标的最大值
        CellLimits cell_limits_;
        Eigen::Vector2d min_; //YH230704
    };


}

#endif //BETA_SUBMAP_LIMIT_H
