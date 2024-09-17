/**
 * @date 24-08-30
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
public:
    explicit MotionFilter(const proto::MotionFilterOptions& options);

    /**
     * @brief 将当前时间与当前位姿 与 上一次保存的时间与位姿进行比对,
     * 时间,移动距离,角度 变换量大于阈值 时返回true
     * @param[in] time 当前的时间
     * @param[in] pose 当前的位姿
     * @return true 两个位姿的时间与距离很接近
     * @return false 两个位姿的时间或距离不接近
     */
    bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

private:
    const proto::MotionFilterOptions options_;
    int num_total_ = 0;
    int num_different_ = 0;
    common::Time last_time_;
    transform::Rigid3d last_pose_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
