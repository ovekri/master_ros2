#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
/*
class PointCloud2Listener : public rclcpp::Node {
public:
  PointCloud2Listener() : Node("lidar") {
    subscription_realsense_pointcloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_transformed", 10, std::bind(&PointCloud2Listener::topic_callback, this, std::placeholders::_1));
    
    //publisher_realsense_control_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(                                                                         // makes a publisher who publish message of type pointcloud2 to topic /LiDAR_transformed
    //    "/realsense_decision", 1);
  }

private:
  bool forward() {
    int counter_tot = 0;
    int counter_approved = 0;
    int counter_to_close = 0;
    double to_close_ratio = 0.0;
    int decision = 0;
    
    for (const auto& point : xyz_point) {
        counter_tot++;
        if (point[2] < 0.2 && point[2] > 0.0) { // height
            if (point[1] < 0.2 && point[1] > -0.2) { // width
                if (point[0] < 10.0 && point[0] > 0.0) { // have to go when outdoor
                    counter_approved++;
                }
                if (point[0] < 2.0 && point[0] > 0.4) { // distance
                    counter_to_close++;
                }
            }
        }
    }
    if (counter_approved != 0) {
        to_close_ratio = static_cast<double>(counter_to_close) / counter_approved;
    }
    if (to_close_ratio < 0.2 && counter_approved < 30) {
        decision = 1;
    }
    else{
        decision = 0;
    }
    return decision
  }

  int turn() {
    
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_realsense_pointcloud2_;
};
*/
int main(/*int argc, char *argv[]*/) {
  /*
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloud2Listener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  */
}
