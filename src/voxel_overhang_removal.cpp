#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct GridXY
{
  int x;
  int y;

  bool operator==(const GridXY & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridXYHash
{
  std::size_t operator()(const GridXY & coord) const
  {
    std::uint64_t h = 1469598103934665603ULL;
    const auto mix = [&h](std::int64_t value) {
      h ^= static_cast<std::uint64_t>(value);
      h *= 1099511628211ULL;
    };
    mix(coord.x);
    mix(coord.y);
    return static_cast<std::size_t>(h);
  }
};

struct ColumnCell
{
  int z{0};
  std::size_t count{0};
};

class VoxelOverhangRemoval : public rclcpp::Node
{
public:
  VoxelOverhangRemoval()
  : Node("voxel_overhang_removal")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/octomap_pointcloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/octomap_pointcloud/filtering");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    height_mode_ = declare_parameter<std::string>("height_mode", "max");
    queue_depth_ = declare_parameter<int>("queue_depth", 1);
    const auto durability = declare_parameter<std::string>("durability", "transient_local");

    if (voxel_size_ <= 0.0) {
      throw std::runtime_error("voxel_size must be positive");
    }
    if (height_mode_ != "max" && height_mode_ != "min") {
      throw std::runtime_error("height_mode must be 'max' or 'min'");
    }

    rclcpp::QoS qos(static_cast<size_t>(std::max(queue_depth_, 1)));
    qos.reliable();
    if (durability == "transient_local") {
      qos.transient_local();
    } else if (durability == "system_default") {
      qos.durability(rclcpp::DurabilityPolicy::SystemDefault);
    } else {
      qos.durability_volatile();
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&VoxelOverhangRemoval::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Subscribing [%s], publishing 2.5D voxel cloud [%s], voxel_size=%.3f, height_mode=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      voxel_size_,
      height_mode_.c_str());
  }

private:
  static int toGrid(double value, double voxel_size)
  {
    return static_cast<int>(std::floor(value / voxel_size));
  }

  static double voxelCenter(int index, double voxel_size)
  {
    return (static_cast<double>(index) + 0.5) * voxel_size;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    std::unordered_map<GridXY, ColumnCell, GridXYHash> columns;
    columns.reserve(input_cloud.size());

    std::size_t finite_points = 0;
    for (const auto & point : input_cloud.points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      ++finite_points;

      const GridXY xy{
        toGrid(point.x, voxel_size_),
        toGrid(point.y, voxel_size_)};
      const int z = toGrid(point.z, voxel_size_);

      auto iter = columns.find(xy);
      if (iter == columns.end()) {
        columns.emplace(xy, ColumnCell{z, 1});
        continue;
      }

      auto & cell = iter->second;
      if ((height_mode_ == "max" && z > cell.z) ||
        (height_mode_ == "min" && z < cell.z))
      {
        cell.z = z;
      }
      ++cell.count;
    }

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    output_cloud.points.reserve(columns.size());
    for (const auto & entry : columns) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(voxelCenter(entry.first.x, voxel_size_));
      point.y = static_cast<float>(voxelCenter(entry.first.y, voxel_size_));
      point.z = static_cast<float>(voxelCenter(entry.second.z, voxel_size_));
      output_cloud.points.push_back(point);
    }

    std::sort(
      output_cloud.points.begin(),
      output_cloud.points.end(),
      [](const pcl::PointXYZ & lhs, const pcl::PointXYZ & rhs) {
        if (lhs.x != rhs.x) {
          return lhs.x < rhs.x;
        }
        if (lhs.y != rhs.y) {
          return lhs.y < rhs.y;
        }
        return lhs.z < rhs.z;
      });

    output_cloud.width = static_cast<std::uint32_t>(output_cloud.points.size());
    output_cloud.height = 1;
    output_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = msg->header;
    if (!frame_id_.empty()) {
      output_msg.header.frame_id = frame_id_;
    }
    output_msg.header.stamp = now();
    publisher_->publish(output_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Published 2.5D cloud: input=%zu finite=%zu output=%zu removed=%zu",
      input_cloud.size(),
      finite_points,
      output_cloud.size(),
      finite_points > output_cloud.size() ? finite_points - output_cloud.size() : 0);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string height_mode_;
  double voxel_size_{0.05};
  int queue_depth_{1};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelOverhangRemoval>());
  rclcpp::shutdown();
  return 0;
}

