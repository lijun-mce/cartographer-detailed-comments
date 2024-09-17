/**
 * @date 24-09-6
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

/**
 * @brief 类Submap2D的对象就是我们一直说的子图。它通过Grid2D保存子图的详细数据
 * Submap 是一个基类，保存了2D 3D共用的一些数据和函数
 * 里面主要使用 Grid2D 存储数据
 */
class Submap2D : public Submap {
public:
    // 主要关心的构造方式
    Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
             ValueConversionTables* conversion_tables);
    explicit Submap2D(const proto::Submap2D& proto, ValueConversionTables* conversion_tables);

    proto::Submap ToProto(bool include_grid_data) const override;
    void UpdateFromProto(const proto::Submap& proto) override;

    void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                         proto::SubmapQuery::Response* response) const override;

    const Grid2D* grid() const { return grid_.get(); }

    // Insert 'range_data' into this submap using 'range_data_inserter'. The
    // submap must not be finished yet.
    void InsertRangeData(const sensor::RangeData& range_data,
                         const RangeDataInserterInterface* range_data_inserter);
    void Finish();

private:
    // 栅格数据
    std::unique_ptr<Grid2D> grid_;

    // 转换表, 第[0-32767]位置, 存的是[0.9, 0.1~0.9]的数据
    ValueConversionTables* conversion_tables_;
};

/**
 * @brief 实际上只用到了"旧图"和"新图"两个子图。其中旧图用于扫描匹配，新图作为储备。
 * 当新图中插入的数据达到一定程度之后，就替换旧图进行扫描匹配，并重新创建一个新图
 * 2个活跃的子图,旧的用于匹配,新的用于插入数据。当新图中插入一定数量的数据完成了初始化操作之后，它就会被当作旧图，用于扫描匹配
 * 只有初始化时才只有1个子图.
 */
class ActiveSubmaps2D {
public:
    // trajectory_builder_2d.lua 参数文件中
    explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);
    ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
    ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

    // 将点云数据写入到submap中 ******* ======== 主函数 ==================
    std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
        const sensor::RangeData& range_data);

    // 获取所有的子图
    std::vector<std::shared_ptr<const Submap2D>> submaps() const;

private:
    // 创建地图数据插入器
    std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();

    std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
    void FinishSubmap();
    void AddSubmap(const Eigen::Vector2f& origin);

    // 子图的配置选项
    const proto::SubmapsOptions2D options_;
    // 保存当前维护子图的容器
    std::vector<std::shared_ptr<Submap2D>> submaps_;
    // 用于将扫描数据插入子图的工具，我们称它为插入器
    std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;

    // 转换表, 第[0-32767]位置, 存的是[0.9, 0.1~0.9]的数据
    ValueConversionTables conversion_tables_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
