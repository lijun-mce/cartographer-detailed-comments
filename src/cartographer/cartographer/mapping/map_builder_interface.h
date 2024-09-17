/**
 * @date 24-08-13
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// map_builder 基类
class MapBuilderInterface {
public:
    // 回调函数
    using LocalSlamResultCallback = TrajectoryBuilderInterface::LocalSlamResultCallback;
    using SensorId = TrajectoryBuilderInterface::SensorId;

    MapBuilderInterface() {}
    virtual ~MapBuilderInterface() {}

    MapBuilderInterface(const MapBuilderInterface&) = delete;
    MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

    /**
     * @brief 创建一个新的轨迹跟踪器并返回该跟踪器的索引
     * @param expected_sensor_ids 记录了用于建图的所有传感器名称和类型
     * @param trajectory_options 轨迹跟踪器的配置
     * @param local_slam_result_callback 回调函数对象，用于响应局部地图构建完成的事件
     */
    virtual int AddTrajectoryBuilder(const std::set<SensorId>& expected_sensor_ids,
                                     const proto::TrajectoryBuilderOptions& trajectory_options,
                                     LocalSlamResultCallback local_slam_result_callback) = 0;

    /**
     * @brief 同样也是一个用于新建轨迹跟踪器的接口
     * @param options_with_sensor_ids_proto 轨迹跟踪器的配置 + 传感器的配置
     */
    virtual int AddTrajectoryForDeserialization(
        const proto::TrajectoryBuilderOptionsWithSensorIds& options_with_sensor_ids_proto) = 0;

    // 获取一个索引为trajectory_id的轨迹跟踪器对象
    virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(int trajectory_id) const = 0;

    // 关闭trajectory_id对应的轨迹跟踪器，该跟踪器将不再响应新的传感器数据
    virtual void FinishTrajectory(int trajectory_id) = 0;

    // 用于将submap_id所对应的子图信息填充到response中。如果出错将错误信息以字符串的形式返回，若成功运行则返回空字符串
    virtual std::string SubmapToProto(const SubmapId& submap_id,
                                      proto::SubmapQuery::Response* response) = 0;

    // 将当前的系统状态转换成一个proto的流，完成序列化
    virtual void SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* writer) = 0;

    // 将当前的系统状态转换成一个 filename，保存到文件中
    virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) = 0;

    // SerializeState的逆操作，用于从proto流中加载SLAM状态
    virtual std::map<int /* trajectory id in proto */, int /* trajectory id */> LoadState(
        io::ProtoStreamReaderInterface* reader, bool load_frozen_state) = 0;

    // SerializeStateToFile的逆操作，用于从文件中加载SLAM状态
    virtual std::map<int /* trajectory id in proto */, int /* trajectory id */> LoadStateFromFile(
        const std::string& filename, bool load_frozen_state) = 0;

    // 获取当前轨迹跟踪器的数量
    virtual int num_trajectory_builders() const = 0;

    // 用于实现闭环检测的PoseGraph对象
    virtual mapping::PoseGraphInterface* pose_graph() = 0;

    // 获取所有轨迹跟踪器的配置
    virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
    GetAllTrajectoryBuilderOptions() const = 0;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
