/**
 * @date 24-09-10
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

proto::ProbabilityGridRangeDataInserterOptions2D CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
public:
    explicit ProbabilityGridRangeDataInserter2D(
        const proto::ProbabilityGridRangeDataInserterOptions2D& options);

    ProbabilityGridRangeDataInserter2D(const ProbabilityGridRangeDataInserter2D&) = delete;
    ProbabilityGridRangeDataInserter2D& operator=(const ProbabilityGridRangeDataInserter2D&) =
        delete;

    // Inserts 'range_data' into 'probability_grid'.
    virtual void Insert(const sensor::RangeData& range_data, GridInterface* grid) const override;

private:
    // 配置文件
    const proto::ProbabilityGridRangeDataInserterOptions2D options_;
    // 更新占用栅格的表
    const std::vector<uint16> hit_table_;
    const std::vector<uint16> miss_table_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
