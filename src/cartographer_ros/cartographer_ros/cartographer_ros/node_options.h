/**
 * lijun 20240823
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/trajectory_options.h"

namespace cartographer_ros {

// proto message
// MapBuilderOptions {
//     // 是否是2d建图
//     bool use_trajectory_builder_2d = 1;
//     // 是否是3d建图
//     bool use_trajectory_builder_3d = 2;
//     // 用于后台计算的线程数
//     int32 num_background_threads = 3;
//     PoseGraphOptions pose_graph_options = 4;
//     // 对每个轨迹独立排序传感器输入
//     bool collate_by_trajectory = 5;
// }

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
    // 在参数配置文件 map_builder.lua 中
    ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    // 坐标转换查找超时时间
    double lookup_transform_timeout_sec;
    // 子图发布周期
    double submap_publish_period_sec;
    // 位姿发布周期
    double pose_publish_period_sec;
    // 轨迹发布周期
    double trajectory_publish_period_sec;
    // 发布tf
    bool publish_to_tf = true;
    // 发布位姿信息
    bool publish_tracked_pose = false;
    // 使用位置外推
    bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(const std::string& configuration_directory,
                                                       const std::string& configuration_basename);
} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
