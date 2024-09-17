/**
 * @date 24-08-13
 * @author comment by lijun
 */

#include "cartographer/mapping/map_builder_interface.h"

#include "cartographer/mapping/pose_graph.h"

namespace cartographer {
namespace mapping {

/**
 * @brief 根据lua字典中的参数, 生成protobuf的序列化数据结构
 * @param[in] parameter_dictionary lua字典
 * @return proto::MapBuilderOptions
 */
proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
    proto::MapBuilderOptions options;
    options.set_use_trajectory_builder_2d(
        parameter_dictionary->GetBool("use_trajectory_builder_2d"));
    options.set_use_trajectory_builder_3d(
        parameter_dictionary->GetBool("use_trajectory_builder_3d"));
    options.set_num_background_threads(
        parameter_dictionary->GetNonNegativeInt("num_background_threads"));
    options.set_collate_by_trajectory(parameter_dictionary->GetBool("collate_by_trajectory"));
    *options.mutable_pose_graph_options() =
        CreatePoseGraphOptions(parameter_dictionary->GetDictionary("pose_graph").get());
    CHECK_NE(options.use_trajectory_builder_2d(), options.use_trajectory_builder_3d());
    return options;
}

} // namespace mapping
} // namespace cartographer
