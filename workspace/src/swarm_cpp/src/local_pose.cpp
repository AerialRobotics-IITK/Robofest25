#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <vector>
#include <array>
#include <cstdlib>
#include <chrono>
#include <cmath>

class HomePositionNode : public rclcpp::Node
{
public:
    HomePositionNode() : Node("home_position_node")
    {
        // Get namespace from environment variable
        const char* mav_id_env = std::getenv("MAV_ID");
        int mav_id = mav_id_env ? std::stoi(mav_id_env) : 2;
        namespace_ = "uav" + std::to_string(mav_id);
        
        // Define QoS profiles
        rclcpp::QoS gps_qos_profile(rclcpp::KeepLast(10));
        gps_qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        gps_qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
        
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Publishers
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/" + namespace_ + "/local_pos", qos_profile);
            
        offset_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/" + namespace_ + "/offset", gps_qos_profile);
        
        // Subscribers
        uav1_global_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/uav1/global_position/global", qos_profile,
            std::bind(&HomePositionNode::uav1GlobalCallback, this, std::placeholders::_1));
        
        own_global_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/" + namespace_ + "/global_position/global", qos_profile,
            std::bind(&HomePositionNode::ownGlobalCallback, this, std::placeholders::_1));
        
        own_local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + namespace_ + "/local_position/pose", qos_profile,
            std::bind(&HomePositionNode::ownLocalCallback, this, std::placeholders::_1));
        
        // Initialize state flags
        global_averaging_complete_ = false;
        global_start_time_set_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Home position node started for %s", namespace_.c_str());
        RCLCPP_INFO(this->get_logger(), "Collecting global positions for offset calculation...");
    }

private:
    void uav1GlobalCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (global_averaging_complete_) return;
        
        if (!global_start_time_set_) {
            global_start_time_ = this->now();
            global_start_time_set_ = true;
        }
        
        auto current_time = this->now();
        double elapsed = (current_time - global_start_time_).seconds();
        
        if (elapsed <= 5.0) {
            uav1_global_positions_.push_back({msg->latitude, msg->longitude, msg->altitude});
        }
    }
    
    void ownGlobalCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (global_averaging_complete_) return;
        
        if (!global_start_time_set_) {
            global_start_time_ = this->now();
            global_start_time_set_ = true;
        }
        
        auto current_time = this->now();
        double elapsed = (current_time - global_start_time_).seconds();
        
        if (elapsed <= 5.0) {
            own_global_positions_.push_back({msg->latitude, msg->longitude, msg->altitude});
        }
        
        if (elapsed > 5.0 && !global_averaging_complete_) {
            if (!uav1_global_positions_.empty() && !own_global_positions_.empty()) {
                global_averaging_complete_ = true;
                offset_position_ = calculateOffsetVector();
                RCLCPP_INFO(this->get_logger(), "Offset calculation complete: x=%f, y=%f, z=%f",
                    offset_position_->point.x, offset_position_->point.y, offset_position_->point.z);
            } else {
                RCLCPP_WARN(this->get_logger(), "Insufficient global position data from one or both drones");
            }
        }
    }
    
    void ownLocalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double current_x = msg->pose.position.x;
        double current_y = msg->pose.position.y;
        double current_z = msg->pose.position.z;
        
        if (offset_position_ != nullptr) {
            // Note: Python comment says "subtract" but code adds offset
            current_x += offset_position_->point.x;
            current_y += offset_position_->point.y;
            current_z += offset_position_->point.z;
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Offset Not set for %s", namespace_.c_str());
            return;
        }
        
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = "map";
        point_msg.point.x = current_x;
        point_msg.point.y = current_y;
        point_msg.point.z = current_z;
        local_pos_pub_->publish(point_msg);
        
        if (offset_position_ != nullptr) {
            offset_pub_->publish(*offset_position_);
        }
    }
    
    geometry_msgs::msg::PointStamped::SharedPtr calculateOffsetVector()
    {
        if (uav1_global_positions_.empty() || own_global_positions_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Missing global position data for offset calculation!");
            return nullptr;
        }
        
        // Calculate average GPS positions for both drones
        std::array<double, 3> avg_uav1_gps = {0.0, 0.0, 0.0};
        for (const auto& pos : uav1_global_positions_) {
            avg_uav1_gps[0] += pos[0];
            avg_uav1_gps[1] += pos[1];
            avg_uav1_gps[2] += pos[2];
        }
        avg_uav1_gps[0] /= uav1_global_positions_.size();
        avg_uav1_gps[1] /= uav1_global_positions_.size();
        avg_uav1_gps[2] /= uav1_global_positions_.size();
        
        std::array<double, 3> avg_own_gps = {0.0, 0.0, 0.0};
        for (const auto& pos : own_global_positions_) {
            avg_own_gps[0] += pos[0];
            avg_own_gps[1] += pos[1];
            avg_own_gps[2] += pos[2];
        }
        avg_own_gps[0] /= own_global_positions_.size();
        avg_own_gps[1] /= own_global_positions_.size();
        avg_own_gps[2] /= own_global_positions_.size();
        
        // Calculate geodesic distance and azimuth from uav1 to own position
        GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), 
                                     GeographicLib::Constants::WGS84_f());
        
        double lat1 = avg_uav1_gps[0];
        double lon1 = avg_uav1_gps[1];
        double lat2 = avg_own_gps[0];
        double lon2 = avg_own_gps[1];
        
        double s12;  // distance
        double azi1; // azimuth from point 1 to point 2
        double azi2; // azimuth from point 2 to point 1 (not used)
        
        geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
        
        double azimuth_rad = azi1 * M_PI / 180.0;
        
        double x_offset = s12 * std::sin(azimuth_rad);
        double y_offset = s12 * std::cos(azimuth_rad);
        double z_offset = avg_own_gps[2] - avg_uav1_gps[2];
        
        auto point_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        point_msg->header.stamp = this->now();
        point_msg->header.frame_id = "map";
        point_msg->point.x = x_offset;
        point_msg->point.y = y_offset;
        point_msg->point.z = z_offset;
        
        return point_msg;
    }
    
    // Member variables
    std::string namespace_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr offset_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav1_global_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr own_global_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr own_local_sub_;
    
    std::vector<std::array<double, 3>> uav1_global_positions_;
    std::vector<std::array<double, 3>> own_global_positions_;
    geometry_msgs::msg::PointStamped::SharedPtr offset_position_;
    
    rclcpp::Time global_start_time_;
    bool global_start_time_set_;
    bool global_averaging_complete_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HomePositionNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Node stopped: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
