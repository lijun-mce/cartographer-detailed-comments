/**
 * @date 24-09-5
 * @author comment by lijun
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
public:
    // 将点云插入到地图后的result
    struct InsertionResult {
        // 插入节点的数据
        std::shared_ptr<const TrajectoryNode::Data> constant_data;
        // 被插入的子图
        std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; // 最多只有2个子图的指针
    };
    // 扫描匹配的result
    struct MatchingResult {
        // 扫描匹配发生的时间
        common::Time time;
        // 在局部地图坐标系下的位姿
        transform::Rigid3d local_pose;
        // 经过扫描匹配之后位姿校准之后的雷达数据
        sensor::RangeData range_data_in_local;
        // 'nullptr' if dropped by the motion filter. 子图插入的结果（失败为空指针）
        std::unique_ptr<const InsertionResult> insertion_result;
    };

    explicit LocalTrajectoryBuilder2D(const proto::LocalTrajectoryBuilderOptions2D& options,
                                      const std::vector<std::string>& expected_range_sensor_ids);
    ~LocalTrajectoryBuilder2D();

    LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
    LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

    // Returns 'MatchingResult' when range data accumulation completed,
    // otherwise 'nullptr'. Range data must be approximately horizontal
    // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
    // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
    // relative time of point with respect to `TimedPointCloudData::time`.
    // 核心功能，在这里进行扫描匹配, 将点云写成地图数据 返回匹配结果
    std::unique_ptr<MatchingResult> AddRangeData(const std::string& sensor_id,
                                                 const sensor::TimedPointCloudData& range_data);
    void AddImuData(const sensor::ImuData& imu_data);
    void AddOdometryData(const sensor::OdometryData& odometry_data);

    static void RegisterMetrics(metrics::FamilyFactory* family_factory);

private:
    std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
        common::Time time, const sensor::RangeData& gravity_aligned_range_data,
        const transform::Rigid3d& gravity_alignment,
        const absl::optional<common::Duration>& sensor_duration);
    sensor::RangeData TransformToGravityAlignedFrameAndFilter(
        const transform::Rigid3f& transform_to_gravity_aligned_frame,
        const sensor::RangeData& range_data) const;
    std::unique_ptr<InsertionResult> InsertIntoSubmap(
        common::Time time, const sensor::RangeData& range_data_in_local,
        const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
        const transform::Rigid3d& pose_estimate, const Eigen::Quaterniond& gravity_alignment);

    // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
    // observed pose, or nullptr on failure.
    std::unique_ptr<transform::Rigid2d> ScanMatch(
        common::Time time, const transform::Rigid2d& pose_prediction,
        const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

    // 初始化一个位姿估计器
    void InitializeExtrapolator(common::Time time);

    // 轨迹跟踪器的配置选项
    const proto::LocalTrajectoryBuilderOptions2D options_;
    // 当前正在维护的子图
    ActiveSubmaps2D active_submaps_;
    // 运动滤波器，对位姿相关的数据进行降采样
    MotionFilter motion_filter_;
    // 实时相关性分析的扫描匹配器  算法"Real-Time Correlative Scan Matching"的实现
    scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
    // 使用Ceres库将扫描数据放置到地图中的扫描匹配器
    scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
    // 位姿估计器，用一段时间内的位姿数据估计线速度和角速度，进而预测运动
    std::unique_ptr<PoseExtrapolator> extrapolator_;
    // 累计数据的数量
    int num_accumulated_ = 0;
    // 累计的扫描数据
    sensor::RangeData accumulated_range_data_;
    // 开始累积数据的时间，也是开始跟踪轨迹的时间
    absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
    absl::optional<double> last_thread_cpu_time_seconds_;
    absl::optional<common::Time> last_sensor_time_;
    // 累积数据收集器
    RangeDataCollator range_data_collator_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
