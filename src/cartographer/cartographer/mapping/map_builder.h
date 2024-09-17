/**
 * @date 24-08-30
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 的完整的SLAM
class MapBuilder : public MapBuilderInterface {
public:
    explicit MapBuilder(const proto::MapBuilderOptions &options);
    ~MapBuilder() override {}

    MapBuilder(const MapBuilder &) = delete;
    MapBuilder &operator=(const MapBuilder &) = delete;

    /**
     * @brief 创建一个新的轨迹跟踪器并返回该跟踪器的索引
     * @param expected_sensor_ids 中记录了用于建图的所有传感器名称和类型
     * @param trajectory_options 新建的轨迹跟踪器的配置
     * @param local_slam_result_callback 是一个回调函数对象，用于响应局部地图构建完成的事件
     * @return int 返回跟踪器的索引
     */
    int AddTrajectoryBuilder(const std::set<SensorId> &expected_sensor_ids,
                             const proto::TrajectoryBuilderOptions &trajectory_options,
                             LocalSlamResultCallback local_slam_result_callback) override;

    // 用于新建轨迹跟踪器的接口
    int AddTrajectoryForDeserialization(
        const proto::TrajectoryBuilderOptionsWithSensorIds &options_with_sensor_ids_proto) override;

    // 关闭轨迹跟踪器
    void FinishTrajectory(int trajectory_id) override;

    // 将submap_id所对应的子图信息填充到proto流中
    std::string SubmapToProto(const SubmapId &submap_id,
                              proto::SubmapQuery::Response *response) override;

    // 将系统状态转换为proto流，进行序列化
    void SerializeState(bool include_unfinished_submaps,
                        io::ProtoStreamWriterInterface *writer) override;

    // 将系统状态转换为file文件
    bool SerializeStateToFile(bool include_unfinished_submaps,
                              const std::string &filename) override;

    // 从proto流中加载系统状态
    std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                                 bool load_frozen_state) override;
    // 从文件中加载系统状态
    std::map<int, int> LoadStateFromFile(const std::string &filename,
                                         const bool load_frozen_state) override;

    // 取用于实现闭环检测的PoseGraph对象
    mapping::PoseGraphInterface *pose_graph() override { return pose_graph_.get(); }

    // 获取当前轨迹跟踪器的数量
    int num_trajectory_builders() const override { return trajectory_builders_.size(); }

    // 返回指向CollatedTrajectoryBuilder的指针
    mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(int trajectory_id) const override {
        return trajectory_builders_.at(trajectory_id).get();
    }

    // 获取所有的轨迹跟踪器的配置
    const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> &
    GetAllTrajectoryBuilderOptions() const override {
        return all_trajectory_builder_options_;
    }

private:
    // 用于记录运行配置
    const proto::MapBuilderOptions options_;
    // 线程池 用于方便高效的管理多线程
    common::ThreadPool thread_pool_;
    // 该对象用于在后台完成闭环检测，进行全局的地图优化   后端
    std::unique_ptr<PoseGraph> pose_graph_;
    // 用来管理和收集传感器数据
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
    //  用于在前台构建子图, 针对每一条轨迹Cartographer都建立了一个轨迹跟踪器  前端
    std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>> trajectory_builders_;
    // 记录了所有轨迹跟踪器的配置(和trajectory_builders_应该是一一对应的关系)
    std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> all_trajectory_builder_options_;
};

// 工厂函数
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(const proto::MapBuilderOptions &options);

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
