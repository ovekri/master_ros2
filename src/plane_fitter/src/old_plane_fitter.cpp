#include <fstream>
#include <memory>
#include <chrono>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

class ColorMap{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B){
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }


private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};

class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

class pointCloudPlaneFitter : public rclcpp::Node {
    public:
        std::string name_;
        pointCloudPlaneFitter() : Node("point_cloud_plane_fitter") {
            name_ = this->get_name();
            publisher_feature_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/plane_fitter", 1);
            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/realsense_transformed", 1, 
                std::bind(&pointCloudPlaneFitter::pointCloudCb, this, std::placeholders::_1));
            
            max_distance_ = 0.035;

            createColors();

            RCLCPP_INFO(get_logger(), "%s: node initialized.", name_.c_str());
        }
                            
        double getPercentile(std::vector<double> vector, float percentile) {
            size_t size = vector.size();                                                                                                                        // retrieves the size of the vector
            sort(vector.begin(), vector.end());                                                                                                                 // sorts the elements of the vector
            return vector[int(size * percentile/100.0)];
        }

        void pointCloudSplitter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            
        }

        void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // Convert to pcl pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_msg);
            RCLCPP_INFO(get_logger(), "%s: new pointcloud (%i, %i)(%zu)", name_.c_str(), cloud_msg->width, cloud_msg->height, cloud_msg->size());
            // Filter cloud
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud_msg);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits(0.001, 10000);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pass.filter (*cloud);
            // Get segmentation ready
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold(max_distance_);
            //Create pointcloud to publish inliers
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
            int original_size(cloud->height*cloud->width);
            // Fit the plane
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            // Iterate inliers to get error
            double mean_error(0);
            double MSE(0);
            double max_error(0);
            double min_error(100000);
            std::vector<double> err;
            for (size_t i = 0; i < inliers->indices.size(); i++) {
                // Get Point
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];
                // Compute distance
                double d = point2planedistance(pt, coefficients)*1000;
                err.push_back(d);
                // Update statistics
                mean_error += d;
                MSE += pow(d, 2);
                if (d > max_error) max_error = d;
                if (d < min_error) min_error = d;
            }
            mean_error /= inliers->indices.size();
            MSE /= inliers->indices.size();
            // Compute Standard deviation
            ColorMap cm(min_error,max_error);
            double sigma(0);
            for (size_t i=0;i<inliers->indices.size();i++){
                sigma += pow(err[i] - mean_error,2);
            }
            sigma = sqrt(sigma/inliers->indices.size());

            RCLCPP_INFO(get_logger(), "%s: me: %lu points, %.2f(mm), mse: %.2f, sd: %.2f (mm), %.1f%% of points",name_.c_str(),inliers->indices.size(),mean_error,MSE,sigma,(double(inliers->indices.size()) / double(original_size))*100.0);
            if (logFile_.is_open()) {
                if (firstPrint_) {
                    logFile_ << "mean, mse, sd, inliers\n";
                    firstPrint_ = false;
                } else {
                    logFile_ << "\n";
                }
                if (inliers->indices.size() < 5){
                    logFile_ << "0.0, 0.0, 0.0, " << inliers->indices.size();
                } else {

                    logFile_ << std::to_string(mean_error) << ", "
                            << std::to_string(MSE) << ", "
                            << std::to_string(sigma) << ", "
                            << inliers->indices.size();
                }
            }
            std_msgs::msg::Float64MultiArray featureMsg;
            // set up dimensions
            featureMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            featureMsg.layout.dim[0].size = 4;
            featureMsg.layout.dim[0].stride = 1;
            featureMsg.layout.dim[0].label = "x";
            // copy in the data
            featureMsg.data.clear();
            featureMsg.data.resize(4);
            if (inliers->indices.size() < 5){
                for (size_t i = 0; i < featureMsg.data.size(); i++){
                    featureMsg.data[i] = 0.0;
                }
            } else {
                featureMsg.data[0] = mean_error;
                featureMsg.data[1] = MSE;
                featureMsg.data[2] = sigma;
                featureMsg.data[3] = inliers->indices.size();
            }
            publisher_feature_->publish(featureMsg);
        }

        void createColors(){
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            for (int i=0;i<20;i++){
                while (r<70 && g < 70 && b < 70){
                    r = rand()%(255);
                    g = rand()%(255);
                    b = rand()%(255);
                }
                Color c(r,g,b);
                r = 0;
                g = 0;
                b = 0;
                colors.push_back(c);
            }
        }
        
        void spin() {
            rclcpp::spin(shared_from_this());
        }

    private:

        double max_distance_;
        std::ofstream logFile_;
        bool firstPrint_;
        std::vector<Color> colors;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_feature_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointCloudPlaneFitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
