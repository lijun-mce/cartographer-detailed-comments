/**
 * @date 24-08-30
 * @author comment by lijun
 */

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
    proto::MotionFilterOptions options;
    options.set_max_time_seconds(parameter_dictionary->GetDouble("max_time_seconds"));
    options.set_max_distance_meters(parameter_dictionary->GetDouble("max_distance_meters"));
    options.set_max_angle_radians(parameter_dictionary->GetDouble("max_angle_radians"));
    return options;
}

MotionFilter::MotionFilter(const proto::MotionFilterOptions& options) : options_(options) {}

/**
 * @brief 将当前时间与当前位姿 与 上一次保存的时间与位姿进行比对,
 * 时间,移动距离,角度 变换量大于阈值 时返回true
 * @param[in] time 当前的时间
 * @param[in] pose 当前的位姿
 * @return true 两个位姿的时间与距离很接近
 * @return false 两个位姿的时间或距离不接近
 */
bool MotionFilter::IsSimilar(const common::Time time, const transform::Rigid3d& pose) {
    LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500) << "Motion filter reduced the number of nodes to "
                                                 << 100. * num_different_ / num_total_ << "%.";
    ++num_total_;
    // max_time_seconds 5s  max_distance_meters 0.2m    max_angle_radians 1弧度
    if (num_total_ > 1 && time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&
        (pose.translation() - last_pose_.translation()).norm() <= options_.max_distance_meters() &&
        transform::GetAngle(pose.inverse() * last_pose_) <= options_.max_angle_radians()) {
        return true;
    }
    // 只有时间,移动距离,角度 变换量大于阈值 才进行 last_pose_ 的更新
    last_time_ = time;
    last_pose_ = pose;
    ++num_different_;
    return false;
}

} // namespace mapping
} // namespace cartographer
