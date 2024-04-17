#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloud2Listener : public rclcpp::Node {
public:
  PointCloud2Listener() : Node("realsense_control") {
    subscription_realsense_pointcloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "realsense_transformed", 10, std::bind(&PointCloud2Listener::topic_callback, this, std::placeholders::_1));
    
    //publisher_realsense_control_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(                                                                         // makes a publisher who publish message of type pointcloud2 to topic /LiDAR_transformed
    //    "/realsense_decision", 1);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    // Example region: x between 1 and 2, y between 1 and 2, z between 1 and 2
    double x_min = 0.5, x_max = 2.0;
    double y_min = -0.15, y_max = 0.15;
    double z_min = 0.0, z_max = 0.2;
    int count = 0;

    // Use the PointCloud2Iterator to access data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (*iter_x >= x_min && *iter_x <= x_max &&
          *iter_y >= y_min && *iter_y <= y_max &&
          *iter_z >= z_min && *iter_z <= z_max) {
        count++;
      }
    }

    //uint32_t total_points = msg->width * msg->height;
    //RCLCPP_INFO(this->get_logger(), "Total number of points: %u", total_points);

    RCLCPP_INFO(this->get_logger(), "Points within the specified area: %d", count);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_realsense_pointcloud2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloud2Listener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
