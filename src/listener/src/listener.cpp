#include <rclcpp/rclcpp.hpp>                                                                                                                        // essential for writing ROS nodes
#include <sensor_msgs/msg/point_cloud2.hpp>                                                                                                         // the definition for the message type pointvloud2
#include <tf2_ros/transform_listener.h>                                                                                                             // used to listen for and receive transformations broadcasted 
#include <tf2_ros/buffer.h>                                                                                                                         // stores the transformation received 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>                                                                                                    // for converting message types
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>                                                                                                        // convert pointcloud2 messages

using std::placeholders::_1;

class Transformer : public rclcpp::Node {                                                                                                           // making a class named Transformer that inherits from rclcpp::Node, making Transformer a ROS node
public:
    Transformer() : Node("transformer") {                                                                                                           // The constructor initializes the Transformer node
        RCLCPP_INFO(this->get_logger(), "Transformer node started.");

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(                                                                   // subscribes to the topic /velodyne_points where it recives pointcloud2 messages.
            "/velodyne_points",
            10,                                                                                                                                      // max queue size
            std::bind(&Transformer::listener_callback, this, _1));                                                                                  // bind the listener_callback to be called every time a new message is received.

        RCLCPP_INFO(this->get_logger(), "Subscribed to /velodyne_points.");

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(                                                                         // makes a publisher who publish message of type pointcloud2 to topic /LiDAR_transformed
            "/LiDAR_transformed", 1);

        RCLCPP_INFO(this->get_logger(), "Publishing to /LiDAR_transformed.");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());                                                                          // stores the transformation received and initializes it with the nodes clock
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);                                                                   // listen to transformations published and stores it in the buffer 

        RCLCPP_INFO(this->get_logger(), "TransformListener initialized.");
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        static bool successful_publish = false;

        auto to_frame = "transformed_lidar_frame";                                                                                                  // frame pointcloud should be transformed to
        auto from_frame = "original_lidar_frame"; // msg->header.frame_id (the velodyne publishes to /tf_static i guess therfor i need to hardcode)                                                                                                     // original frame which is going to be transformed 

        //RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message.");

        geometry_msgs::msg::TransformStamped transform;                                                                                             // tries to lookup the transfomation
        try {
            transform = tf_buffer_->lookupTransform(
                to_frame,
                from_frame,
                msg->header.stamp);                                                                                                           

            //RCLCPP_INFO(this->get_logger(), "Transform from %s to %s found.", from_frame, to_frame);

        } catch (tf2::TransformException& ex) {                                                                                                     // if the transformation not available, throw and exeption with the error and exit
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", to_frame, from_frame, ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform);                                                                                       // transform the pointcloud2 message

        //RCLCPP_INFO(this->get_logger(), "PointCloud2 transformed successfully.");

        publisher_->publish(transformed_cloud);                                                                                                     // publishes the new pointcloud2 message

        if (!successful_publish) {
            RCLCPP_INFO(this->get_logger(), "Transformed PointCloud2 published.");
            successful_publish = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Transformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// TODO: 
