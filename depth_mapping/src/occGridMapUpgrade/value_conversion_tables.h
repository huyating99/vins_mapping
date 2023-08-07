//
// Created by lao-tang on 2023/3/28.
//

#ifndef BETA_VALUE_CONVERSION_TABLES_H
#define BETA_VALUE_CONVERSION_TABLES_H

#include <map>
#include <vector>
#include <memory>

#include <limits>


namespace mapping{
// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.
// 以将 uint16 值映射到 ['lower_bound', 'upper_bound'] 中的浮点数
// 表的第一个元素设置为 unknown_result
    class ValueConversionTables {
    public:
        const std::vector<float>* GetConversionTable(float unknown_result,
                                                     float lower_bound,
                                                     float upper_bound);

    private:
        std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                float /* upper_bound */>,
                std::unique_ptr<const std::vector<float>>>
                bounds_to_lookup_table_;
    };
}

#endif //BETA_VALUE_CONVERSION_TABLES_H
