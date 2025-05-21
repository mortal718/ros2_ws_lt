#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <cmath>
#include <sensor_msgs/point_field_conversion.hpp>

class LaserToPointCloudNode : public rclcpp::Node
{
public:
  LaserToPointCloudNode() : Node("laser_to_pointcloud")
  {
    // 创建订阅者
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/merged", 10, std::bind(&LaserToPointCloudNode::scanCallback, this, std::placeholders::_1));

    // 创建发布者
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/laser_cloud", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header = scan->header;

    // 遍历激光扫描的每个点
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      // 如果距离在有效范围内
      if (scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max)
      {
        // 计算当前角度
        float angle = scan->angle_min + i * scan->angle_increment;
        
        // 将极坐标转换为笛卡尔坐标
        geometry_msgs::msg::Point32 point;
        point.x = scan->ranges[i] * std::cos(angle);
        point.y = scan->ranges[i] * std::sin(angle);
        point.z = 0.0;  // 2D激光雷达，z坐标设为0

        cloud.points.push_back(point);
      }
    }

    // 发布点云消息
    cloud_pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 
