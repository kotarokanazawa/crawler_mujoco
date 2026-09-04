#include <chrono>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class TruthCloudPublisher : public rclcpp::Node
{
public:
  TruthCloudPublisher()
  : Node("truth_cloud_publisher")
  {
    resolution_ = declare_parameter<double>("resolution", 0.025);
    plane_size_ = declare_parameter<double>("plane_size", 10.0);
    plane_step_ = declare_parameter<double>("plane_step", resolution_);
    publish_period_ = declare_parameter<double>("publish_period", 2.0);
    arena_yaml_ = declare_parameter<std::string>("arena_yaml", "");
    model_root_ = declare_parameter<std::string>("model_root", "");
    default_object_size_x_ = declare_parameter<double>("default_object_size_x", 0.8);
    default_object_size_y_ = declare_parameter<double>("default_object_size_y", 0.8);
    default_object_size_z_ = declare_parameter<double>("default_object_size_z", 0.4);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "octomap_pointcloud", rclcpp::QoS(1).transient_local().reliable());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_)),
      std::bind(&TruthCloudPublisher::publishCloudMap, this));

    arena_objects_ = loadArenaObjects(arena_yaml_);
  }

private:
  struct ArenaObject
  {
    struct Position
    {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    };

    std::string base_name;
    Position position;
    double yaw{0.0};
    double roll{0.0};
    double pitch{0.0};
    double scale_x{1.0};
    double scale_y{1.0};
    double scale_z{1.0};
  };

  struct MeshModel
  {
    bool valid{false};
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
  };

  static std::string trim(const std::string& text)
  {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
  }

  static double getMapNumber(
    const std::string& text, const std::string& key, const double default_value)
  {
    const std::regex pattern(key + R"(\s*:\s*([-+0-9.eE]+))");
    std::smatch match;
    if (std::regex_search(text, match, pattern)) {
      try {
        return std::stod(match[1].str());
      } catch (const std::exception&) {
        return default_value;
      }
    }
    return default_value;
  }

  static std::string baseNameFromObjectName(const std::string& name)
  {
    const auto pos = name.find_last_of('_');
    if (pos == std::string::npos) {
      return name;
    }
    const auto suffix = name.substr(pos + 1);
    return std::all_of(suffix.begin(), suffix.end(), [](const unsigned char c) {
      return std::isdigit(c) != 0;
    }) ? name.substr(0, pos) : name;
  }

  std::vector<ArenaObject> loadArenaObjects(const std::string& path)
  {
    std::vector<ArenaObject> objects;
    if (path.empty()) {
      return objects;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "Could not open arena yaml [%s]", path.c_str());
      return objects;
    }

    bool in_objects = false;
    std::string current_name;
    std::unordered_map<std::string, double> current_values;
    const std::regex block_key_pattern(R"(^\s{4}([A-Za-z0-9_.-]+)\s*:\s*$)");
    const std::regex inline_pattern(R"(^\s{4}([A-Za-z0-9_.-]+)\s*:\s*\{(.*)\}\s*$)");
    const std::regex value_pattern(R"(^\s{6}([A-Za-z0-9_.-]+)\s*:\s*([-+0-9.eE]+).*$)");

    auto flush_current = [&]() {
      if (current_name.empty()) {
        return;
      }
      ArenaObject object;
      object.base_name = baseNameFromObjectName(current_name);
      const auto value = [&](const std::string& key, const double default_value) {
        const auto it = current_values.find(key);
        return it == current_values.end() ? default_value : it->second;
      };
      object.roll = value("roll", 0.0) * M_PI / 180.0;
      object.pitch = value("pitch", 0.0) * M_PI / 180.0;
      object.yaw = value("yaw", 0.0) * M_PI / 180.0;
      object.scale_x = value("scale_x", 1.0);
      object.scale_y = value("scale_y", 1.0);
      object.scale_z = value("scale_z", 1.0);
      object.position = {
        value("x", 0.0), value("y", 0.0), value("z", 0.0)};
      objects.push_back(object);
      current_name.clear();
      current_values.clear();
    };

    std::string line;
    while (std::getline(file, line)) {
      const auto comment_pos = line.find('#');
      if (comment_pos != std::string::npos) {
        line = line.substr(0, comment_pos);
      }
      if (trim(line).empty()) {
        continue;
      }
      if (line.find("objects:") != std::string::npos) {
        in_objects = true;
        continue;
      }
      if (!in_objects) {
        continue;
      }
      if (line.rfind("  wall:", 0) == 0 || line.rfind("  ", 0) != 0) {
        break;
      }

      std::smatch match;
      if (std::regex_match(line, match, inline_pattern)) {
        flush_current();
        ArenaObject object;
        object.base_name = baseNameFromObjectName(match[1].str());
        const std::string values = match[2].str();
        object.roll = getMapNumber(values, "roll", 0.0) * M_PI / 180.0;
        object.pitch = getMapNumber(values, "pitch", 0.0) * M_PI / 180.0;
        object.yaw = getMapNumber(values, "yaw", 0.0) * M_PI / 180.0;
        object.scale_x = getMapNumber(values, "scale_x", 1.0);
        object.scale_y = getMapNumber(values, "scale_y", 1.0);
        object.scale_z = getMapNumber(values, "scale_z", 1.0);
        object.position = {
          getMapNumber(values, "x", 0.0),
          getMapNumber(values, "y", 0.0),
          getMapNumber(values, "z", 0.0)};
        objects.push_back(object);
      } else if (std::regex_match(line, match, block_key_pattern)) {
        flush_current();
        current_name = match[1].str();
      } else if (!current_name.empty() && std::regex_match(line, match, value_pattern)) {
        current_values[match[1].str()] = std::stod(match[2].str());
      }
    }
    flush_current();

    RCLCPP_INFO(get_logger(), "Loaded %zu arena objects from [%s]", objects.size(), path.c_str());
    return objects;
  }

  static std::string readTextFile(const std::string& path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      return "";
    }
    std::ostringstream stream;
    stream << file.rdbuf();
    return stream.str();
  }

  static std::string lowerCopy(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return text;
  }

  static std::array<double, 3> parseVector3(
    const std::string& text, const std::array<double, 3>& default_value)
  {
    std::istringstream stream(text);
    std::array<double, 3> value = default_value;
    stream >> value[0] >> value[1] >> value[2];
    return value;
  }

  static std::array<double, 6> parsePose6(
    const std::string& text, const std::array<double, 6>& default_value)
  {
    std::istringstream stream(text);
    std::array<double, 6> value = default_value;
    stream >> value[0] >> value[1] >> value[2] >> value[3] >> value[4] >> value[5];
    return value;
  }

  static Eigen::Matrix3d rpyRotation(
    const double roll,
    const double pitch,
    const double yaw)
  {
    return
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }

  static std::array<double, 6> parseModelPose(const std::string& sdf)
  {
    std::array<double, 6> pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::regex model_pattern(R"(<model[^>]*>([\s\S]*?)<link[\s>])", std::regex::icase);
    const std::regex pose_pattern(R"(<pose[^>]*>([\s\S]*?)</pose>)", std::regex::icase);
    std::smatch model_match;
    if (!std::regex_search(sdf, model_match, model_pattern)) {
      return pose;
    }

    std::smatch pose_match;
    const std::string model_header = model_match[1].str();
    if (std::regex_search(model_header, pose_match, pose_pattern)) {
      pose = parsePose6(trim(pose_match[1].str()), pose);
    }
    return pose;
  }

  std::string resolveMeshUri(const std::string& uri, const std::string& base_name) const
  {
    const std::string prefix = "model://";
    if (uri.rfind(prefix, 0) == 0) {
      const std::string relative = uri.substr(prefix.size());
      return model_root_ + "/" + relative;
    }
    if (!uri.empty() && uri.front() == '/') {
      return uri;
    }
    return model_root_ + "/" + base_name + "/" + uri;
  }

  void sampleTriangle(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    pcl::PointCloud<pcl::PointXYZ>& out) const
  {
    const double step = std::max(resolution_, 0.03);
    const double max_edge = std::max({(a - b).norm(), (b - c).norm(), (c - a).norm()});
    const int divisions = std::max(1, static_cast<int>(std::ceil(max_edge / step)));
    for (int i = 0; i <= divisions; ++i) {
      for (int j = 0; j <= divisions - i; ++j) {
        const double u = static_cast<double>(i) / static_cast<double>(divisions);
        const double v = static_cast<double>(j) / static_cast<double>(divisions);
        const Eigen::Vector3d p = a + u * (b - a) + v * (c - a);
        out.push_back(pcl::PointXYZ(
          static_cast<float>(p.x()),
          static_cast<float>(p.y()),
          static_cast<float>(p.z())));
      }
    }
  }

  MeshModel loadMeshModel(const std::string& base_name) const
  {
    const auto cached = mesh_cache_.find(base_name);
    if (cached != mesh_cache_.end()) {
      return cached->second;
    }

    MeshModel result;
    if (model_root_.empty()) {
      mesh_cache_[base_name] = result;
      return result;
    }

    const std::string sdf_path = model_root_ + "/" + base_name + "/model.sdf";
    const std::string sdf = readTextFile(sdf_path);
    const std::array<double, 6> model_pose = parseModelPose(sdf);
    const std::regex mesh_pattern(R"(<mesh>([\s\S]*?)</mesh>)", std::regex::icase);
    const std::regex uri_pattern(R"(<uri>([\s\S]*?)</uri>)", std::regex::icase);
    const std::regex scale_pattern(R"(<scale>([\s\S]*?)</scale>)", std::regex::icase);
    std::smatch mesh_match;
    if (!std::regex_search(sdf, mesh_match, mesh_pattern)) {
      mesh_cache_[base_name] = result;
      return result;
    }

    const std::string mesh_block = mesh_match[1].str();
    std::smatch uri_match;
    if (!std::regex_search(mesh_block, uri_match, uri_pattern)) {
      mesh_cache_[base_name] = result;
      return result;
    }
    const std::string mesh_path = resolveMeshUri(trim(uri_match[1].str()), base_name);
    const std::string lower_path = lowerCopy(mesh_path);
    if (lower_path.size() < 4 || lower_path.substr(lower_path.size() - 4) != ".stl") {
      RCLCPP_WARN_ONCE(
        get_logger(), "Mesh [%s] is not STL; using box fallback for unsupported mesh formats",
        mesh_path.c_str());
      mesh_cache_[base_name] = result;
      return result;
    }

    std::array<double, 3> mesh_scale{1.0, 1.0, 1.0};
    std::smatch scale_match;
    if (std::regex_search(mesh_block, scale_match, scale_pattern)) {
      mesh_scale = parseVector3(trim(scale_match[1].str()), mesh_scale);
    }

    pcl::PolygonMesh polygon_mesh;
    if (pcl::io::loadPolygonFileSTL(mesh_path, polygon_mesh) == 0) {
      RCLCPP_WARN(get_logger(), "Failed to load mesh [%s]", mesh_path.c_str());
      mesh_cache_[base_name] = result;
      return result;
    }

    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(polygon_mesh.cloud, vertices);
    for (const auto& polygon : polygon_mesh.polygons) {
      if (polygon.vertices.size() < 3) {
        continue;
      }
      const auto make_vertex = [&](const std::uint32_t index) {
        const auto& p = vertices.points[index];
        return Eigen::Vector3d(
          static_cast<double>(p.x) * mesh_scale[0],
          static_cast<double>(p.y) * mesh_scale[1],
          static_cast<double>(p.z) * mesh_scale[2]);
      };
      const Eigen::Vector3d a = make_vertex(polygon.vertices[0]);
      for (std::size_t i = 1; i + 1 < polygon.vertices.size(); ++i) {
        sampleTriangle(a, make_vertex(polygon.vertices[i]), make_vertex(polygon.vertices[i + 1]),
          *result.local_cloud);
      }
    }

    const Eigen::Matrix3d model_rotation = rpyRotation(model_pose[3], model_pose[4], model_pose[5]);
    const Eigen::Vector3d model_translation(model_pose[0], model_pose[1], model_pose[2]);
    if (model_translation.squaredNorm() > 0.0 || !model_rotation.isIdentity(1e-12)) {
      for (auto& point : result.local_cloud->points) {
        const Eigen::Vector3d local(
          static_cast<double>(point.x),
          static_cast<double>(point.y),
          static_cast<double>(point.z));
        const Eigen::Vector3d transformed = model_translation + model_rotation * local;
        point.x = static_cast<float>(transformed.x());
        point.y = static_cast<float>(transformed.y());
        point.z = static_cast<float>(transformed.z());
      }
    }

    result.valid = !result.local_cloud->empty();
    if (result.valid) {
      RCLCPP_INFO(
        get_logger(), "Loaded mesh cloud [%s] with %zu points",
        base_name.c_str(), result.local_cloud->size());
    }
    mesh_cache_[base_name] = result;
    return result;
  }

  std::array<double, 3> objectSize(const std::string& base_name) const
  {
    if (base_name.find("wall50") != std::string::npos) {
      return {1.2, 0.15, 0.5};
    }
    if (base_name.find("wall100") != std::string::npos) {
      return {1.2, 0.15, 1.0};
    }
    if (base_name.find("krails") != std::string::npos || base_name.find("krail") != std::string::npos) {
      return {1.2, 0.25, 0.35};
    }
    if (base_name.find("pallet15") != std::string::npos) {
      return {1.2, 0.8, 0.15};
    }
    if (base_name.find("pallet30") != std::string::npos) {
      return {1.2, 0.8, 0.30};
    }
    if (base_name.find("bridge") != std::string::npos) {
      return {1.2, 0.6, 0.38};
    }
    if (base_name.find("ramp") != std::string::npos || base_name.find("stair") != std::string::npos) {
      return {1.2, 2.4, 0.6};
    }
    if (base_name.find("stepfield") != std::string::npos) {
      return {1.2, 2.4, 0.35};
    }
    const std::regex step_pattern(R"(([0-9]+)cmstep)");
    std::smatch match;
    if (std::regex_search(base_name, match, step_pattern)) {
      return {1.2, 0.6, std::stod(match[1].str()) / 100.0};
    }
    return {default_object_size_x_, default_object_size_y_, default_object_size_z_};
  }

  void addGroundPlane(pcl::PointCloud<pcl::PointXYZ>& cloud) const
  {
    const double half = plane_size_ * 0.5;
    for (double x = -half; x <= half; x += plane_step_) {
      for (double y = -half; y <= half; y += plane_step_) {
        cloud.push_back(pcl::PointXYZ(x, y, 0.0f));
      }
    }
  }

  void addBoxSurface(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    const ArenaObject::Position& position,
    const double yaw,
    const std::array<double, 3>& size,
    const double scale_x = 1.0,
    const double scale_y = 1.0,
    const double scale_z = 1.0) const
  {
    const float x0 = static_cast<float>(position.x);
    const float y0 = static_cast<float>(position.y);
    const float z0 = static_cast<float>(position.z);
    const float sx = static_cast<float>(std::max(size[0] * scale_x, resolution_));
    const float sy = static_cast<float>(std::max(size[1] * scale_y, resolution_));
    const float sz = static_cast<float>(std::max(size[2] * scale_z, resolution_));
    const float step = std::max(static_cast<float>(resolution_), 0.03f);
    const float hx = sx * 0.5f;
    const float hy = sy * 0.5f;
    const float c = static_cast<float>(std::cos(yaw));
    const float s = static_cast<float>(std::sin(yaw));

    auto push = [&](const float lx, const float ly, const float lz) {
      const float wx = x0 + c * lx - s * ly;
      const float wy = y0 + s * lx + c * ly;
      cloud.push_back(pcl::PointXYZ(wx, wy, z0 + lz));
    };

    for (float x = -hx; x <= hx; x += step) {
      for (float y = -hy; y <= hy; y += step) {
        push(x, y, 0.0f);
        push(x, y, sz);
      }
    }
    for (float z = 0.0f; z <= sz; z += step) {
      for (float x = -hx; x <= hx; x += step) {
        push(x, -hy, z);
        push(x, hy, z);
      }
      for (float y = -hy; y <= hy; y += step) {
        push(-hx, y, z);
        push(hx, y, z);
      }
    }
  }

  void addMeshObject(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    const ArenaObject& object,
    const MeshModel& mesh) const
  {
    const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(object.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(object.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(object.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation(
      object.position.x,
      object.position.y,
      object.position.z);

    for (const auto& point : mesh.local_cloud->points) {
      Eigen::Vector3d local(
        static_cast<double>(point.x) * object.scale_x,
        static_cast<double>(point.y) * object.scale_y,
        static_cast<double>(point.z) * object.scale_z);
      const Eigen::Vector3d world = translation + rotation * local;
      cloud.push_back(pcl::PointXYZ(
        static_cast<float>(world.x()),
        static_cast<float>(world.y()),
        static_cast<float>(world.z())));
    }
  }

  void addArenaObjects(pcl::PointCloud<pcl::PointXYZ>& cloud) const
  {
    for (const auto& object : arena_objects_) {
      const auto mesh = loadMeshModel(object.base_name);
      if (mesh.valid) {
        addMeshObject(cloud, object, mesh);
      } else {
        addBoxSurface(
          cloud, object.position, object.yaw, objectSize(object.base_name),
          object.scale_x, object.scale_y, object.scale_z);
      }
    }
  }

  void publishCloudMap()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "map";
    addGroundPlane(cloud);
    addArenaObjects(cloud);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    cloud_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Published cloud map with %zu points", cloud.size());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<ArenaObject> arena_objects_;
  mutable std::unordered_map<std::string, MeshModel> mesh_cache_;
  std::string arena_yaml_;
  std::string model_root_;
  double resolution_;
  double plane_size_;
  double plane_step_;
  double publish_period_;
  double default_object_size_x_;
  double default_object_size_y_;
  double default_object_size_z_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TruthCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
