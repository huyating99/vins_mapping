//
// Created by lao-tang on 2023/3/27.
//
#include "point_cloud.h"

namespace mapping{
    namespace sensor{
        PointCloud::PointCloud() {}
        PointCloud::PointCloud(std::vector<PointCloud::PointType> points)
                : points_(std::move(points)) {}

// 构造时先拷贝, 再进行移动
        PointCloud::PointCloud(std::vector<PointType> points,
                               std::vector<float> intensities)
                : points_(std::move(points)), intensities_(std::move(intensities)) {
            if (!intensities_.empty()) {
                ICHECK_EQ(points_.size(), intensities_.size());
            }
        }

        size_t PointCloud::size() const { return points_.size(); }
        bool PointCloud::empty() const { return points_.empty(); }

// 返回vector的引用
        const std::vector<PointCloud::PointType>& PointCloud::points() const {
            return points_;
        }
// 返回vector的引用
        const std::vector<float>& PointCloud::intensities() const {
            return intensities_;
        }
        const PointCloud::PointType& PointCloud::operator[](const size_t index) const {
            return points_[index];
        }

        PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }
        PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

        void PointCloud::push_back(PointCloud::PointType value) {
            points_.push_back(std::move(value));
        }

    }
}