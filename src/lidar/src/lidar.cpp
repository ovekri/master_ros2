#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// PLC
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

#include <cmath>
#include <vector>
#include <sstream>

class PointCloud2Listener : public rclcpp::Node {
public:
  PointCloud2Listener() : Node("lidar") {
    subscription_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "LiDAR_transformed", 10, std::bind(&PointCloud2Listener::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(                                                                         // makes a publisher who publish message of type pointcloud2 to topic /LiDAR_transformed
        "/obstacle_distance", 10);
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received data:");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_msg);
    std::vector<float> mean(48, 0.0);
    for (const auto& point : cloud_msg->points) {
      if (point.z > 0.0 && point.z < 0.15) {
        float r = std::sqrt(point.x * point.x + point.y * point.y);
        float theta = std::atan2(point.y, point.x);
        int binIndex = static_cast<int>((theta + 1.0472) / 0.08727); // 5 degree increments, -60 to 60. 
        if (binIndex >= 0 && binIndex < 24) { 
            mean[binIndex * 2] += r;                                 // mean[0] is theta = -60 to -55
            mean[binIndex * 2 + 1] += 1;                             // mean[1] is number of points in the theta range
        }
      }
    }

    std_msgs::msg::Float32MultiArray lengths;
    lengths.data.clear(); 
    for(int i = 0; i < 24; i++) {
      float value = 0;
      int k = 23-i;
      if (mean[k*2+1] != 0) {
        value = mean[k*2]/mean[k*2+1];
      }
      else {
        value = 100; // high value or infinite. no points found = no obstacle 
      } 
      lengths.data.push_back(value);
    }

    publisher_->publish(lengths); 
    /*
    // Creating a string to log the obstacle lengths
    std::ostringstream oss;
    oss << "Obstacle vector from lidar is: [";
    for (size_t j = 0; j < lengths.data.size(); j++) {
        oss << lengths.data[j];
        if (j < lengths.data.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    */
  }

private:
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_lidar_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloud2Listener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
