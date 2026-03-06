#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <algorithm>

#include "human_tracking_controls/PID_Controller/pid_controller.hpp"
#include "human_tracking_controls/Utils/vehicle_utils.hpp"
#include "human_tracking_controls/Utils/odometry_utils.hpp"

using namespace vehicle_utils;

// ================================================================
// ===================== HUMAN TRACKING NODE ======================
// ================================================================

class HumanTrackingNode : public rclcpp::Node
{
public:
    HumanTrackingNode()
        : Node("human_tracking_node"),
          pid_yaw_(0.1, 0.01, 0.1, 0.0, -100.0, 100.0)
    {
        namespace_ = declare_parameter<std::string>("namespace", "/uav3");

        init_qos();
        init_publishers();
        init_subscribers();
        init_services();
        init_timer();

        last_time_ = now();
        last_vision_time_ = now();

        RCLCPP_INFO(
            get_logger(),
            "Human Tracking Node Started (ns=%s)",
            namespace_.c_str());
    }

private:
    // ============================================================
    // ===================== FLIGHT STAGES ========================
    // ============================================================

    enum class FlightStage
    {
        SET_MODE,
        ARM,
        TAKEOFF,
        WAIT_ALTITUDE,
        HOVER,
        YAW_CONTROL,
        LAND
    };

    FlightStage stage_ = FlightStage::SET_MODE;
    FlightStage last_logged_stage_ = FlightStage::SET_MODE;

    // ============================================================
    // ===================== CONSTANTS ============================
    // ===========================================================

    static constexpr float TAKEOFF_ALTITUDE = 1.3f;

    static constexpr float WAIST_THRESHOLD_ENTER = 50.0f;
    static constexpr float WAIST_THRESHOLD_EXIT  = 20.0f;

    static constexpr double MAX_YAW_RATE = 0.5;   // rad/s
    static constexpr double VISION_TIMEOUT = 0.5; // sat TAKEOFF_ALTITUDE = 1.0f;
    // New line
    static constexpr double LATERAL_SPEED = 0.5;  // meters per second
    
    // ============================================================
    // ===================== ROS =================================
    // ============================================================

    std::string namespace_;

    rclcpp::QoS mavros_qos_{1};
    rclcpp::QoS pub_qos_{10};

    rclcpp::TimerBase::SharedPtr control_timer_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;

    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr waist_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lateral_sub_;

    // Services
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

    // ============================================================
    // ===================== STATE ================================
    // ============================================================

    mavros_msgs::msg::State current_state_;

    VehicleOdometry vehicle_odom_;
    VehicleOdometry hover_reference_;

    bool has_odom_ = false;

    double waist_error_ = 0.0;
    double yaw_rate_cmd_ = 0.0;
    // new line
    int lateral_cmd_ = 0;

    PIDController pid_yaw_;

    rclcpp::Time last_time_;
    rclcpp::Time last_vision_time_;

    // ============================================================
    // ===================== INITIALIZATION =======================
    // ============================================================

    void init_qos()
    {
        mavros_qos_ = rclcpp::SensorDataQoS(); // BEST_EFFORT
        pub_qos_ = rclcpp::QoS(10).reliable();
    }

    void init_publishers()
    {
        vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            namespace_ + "/setpoint_velocity/cmd_vel", pub_qos_);

        pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            namespace_ + "/setpoint_position/local", pub_qos_);
    }

    void init_subscribers()
    {
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            namespace_ + "/state", mavros_qos_,
            std::bind(&HumanTrackingNode::state_callback, this, std::placeholders::_1));

        pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            namespace_ + "/local_position/pose", mavros_qos_,
            std::bind(&HumanTrackingNode::pose_callback, this, std::placeholders::_1));

        vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            namespace_ + "/local_position/velocity_local", mavros_qos_,
            std::bind(&HumanTrackingNode::velocity_callback, this, std::placeholders::_1));

        waist_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/waist_angle", 10,
            std::bind(&HumanTrackingNode::waist_callback, this, std::placeholders::_1));
        // new line
        lateral_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/lateral_command", 10,
            std::bind(&HumanTrackingNode::lateral_callback, this, std::placeholders::_1));
    }

    void init_services()
    {
        arm_client_     = create_client<mavros_msgs::srv::CommandBool>(namespace_ + "/cmd/arming");
        mode_client_    = create_client<mavros_msgs::srv::SetMode>(namespace_ + "/set_mode");
        takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(namespace_ + "/cmd/takeoff");

        arm_client_->wait_for_service();
        mode_client_->wait_for_service();
        takeoff_client_->wait_for_service();
    }

    void init_timer()
    {
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&HumanTrackingNode::control_loop, this));
    }

    // ============================================================
    // ===================== CALLBACKS ============================
    // ============================================================

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "State: connected=%d armed=%d mode=%s",
            current_state_.connected,
            current_state_.armed,
            current_state_.mode.c_str());
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vehicle_odom_.position.x = msg->pose.position.x;
        vehicle_odom_.position.y = msg->pose.position.y;
        vehicle_odom_.position.z = msg->pose.position.z;

        vehicle_odom_.quaternion.x = msg->pose.orientation.x;
        vehicle_odom_.quaternion.y = msg->pose.orientation.y;
        vehicle_odom_.quaternion.z = msg->pose.orientation.z;
        vehicle_odom_.quaternion.w = msg->pose.orientation.w;

        odom_utils::quaternionToEuler(
            vehicle_odom_.quaternion,
            vehicle_odom_.orientation);

        has_odom_ = true;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Odom: x=%.2f y=%.2f z=%.2f yaw=%.2f",
            vehicle_odom_.position.x,
            vehicle_odom_.position.y,
            vehicle_odom_.position.z,
            vehicle_odom_.orientation.yaw);
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        vehicle_odom_.velocity.x = msg->twist.linear.x;
        vehicle_odom_.velocity.y = msg->twist.linear.y;
        vehicle_odom_.velocity.z = msg->twist.linear.z;
    }

    void waist_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // tracker publishes -1 when the person is not detected.
        if (msg->data == -1.0f)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waist: no detection (data=%.1f)", msg->data);
            // Do NOT update last_vision_time_ so that vision_alive() can timeout.
            return;
        }

        waist_error_ = msg->data;
        last_vision_time_ = now();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Waist: waist_error=%.1f (vision alive)", waist_error_);
    }

    void lateral_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        lateral_cmd_ = msg->data;
    }

    // ============================================================
    // ===================== CONTROL ==============================
    // ============================================================

    double compute_dt()
    {
        auto t = now();

        double dt = (t - last_time_).seconds();

        if (dt < 0.001 || dt > 0.1)
            dt = 0.01;

        last_time_ = t;

        return dt;
    }

    bool vision_alive()
    {
        return (now() - last_vision_time_).seconds() < VISION_TIMEOUT;
    }

    // ============================================================
    // ===================== MAIN LOOP ============================
    // ============================================================

    void control_loop()
    {
        if (!current_state_.connected || !has_odom_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting: connected=%d has_odom=%d",
                current_state_.connected, has_odom_);
            return;
        }

        if (stage_ != last_logged_stage_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Stage transition: %d -> %d",
                static_cast<int>(last_logged_stage_),
                static_cast<int>(stage_));
            last_logged_stage_ = stage_;
        }

        switch (stage_)
        {
            case FlightStage::SET_MODE:      handle_set_mode(); break;
            case FlightStage::ARM:           handle_arm(); break;
            case FlightStage::TAKEOFF:       handle_takeoff(); break;
            case FlightStage::WAIT_ALTITUDE: handle_wait_altitude(); break;
            case FlightStage::HOVER:         handle_hover(); break;
            case FlightStage::YAW_CONTROL:   handle_yaw_control(); break;
            case FlightStage::LAND:          handle_land(); break;
        }
    }

    // ============================================================
    // ===================== STAGES ===============================
    // ============================================================

    void handle_set_mode()
    {
        if (current_state_.mode != "GUIDED")
        {
            RCLCPP_INFO(get_logger(), "SET_MODE: requesting GUIDED");
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "GUIDED";
            mode_client_->async_send_request(req);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "SET_MODE: already GUIDED -> ARM");
            stage_ = FlightStage::ARM;
        }
    }

    void handle_arm()
    {
        if (!current_state_.armed)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "ARM: sending arm command");
            auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            req->value = true;
            arm_client_->async_send_request(req);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "ARM: already armed -> TAKEOFF");
            stage_ = FlightStage::TAKEOFF;
        }
    }

    void handle_takeoff()
    {
        RCLCPP_INFO(
            get_logger(),
            "TAKEOFF: requesting altitude %.2f m",
            TAKEOFF_ALTITUDE);
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = TAKEOFF_ALTITUDE;

        takeoff_client_->async_send_request(req);

        stage_ = FlightStage::WAIT_ALTITUDE;
    }

    void handle_wait_altitude()
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "WAIT_ALTITUDE: z=%.2f (target=%.2f)",
            vehicle_odom_.position.z, TAKEOFF_ALTITUDE);

        if (vehicle_odom_.position.z >= TAKEOFF_ALTITUDE - 0.25)
        {
            hover_reference_ = vehicle_odom_;
            RCLCPP_INFO(
                get_logger(),
                "WAIT_ALTITUDE: reached -> HOVER (z=%.2f)",
                vehicle_odom_.position.z);
            stage_ = FlightStage::HOVER;
        }
    }

    void handle_hover()
    {
        publish_yaw_velocity(0.0);

        // Only allow entering YAW_CONTROL when vision is currently alive.
        if ((vision_alive() && std::abs(waist_error_) > WAIST_THRESHOLD_ENTER) || lateral_cmd_ != 0)
        {
            pid_yaw_.reset();
            RCLCPP_INFO(
                get_logger(),
                "HOVER: |waist_error|=%.1f > %.1f or lateral_cmd=%d -> YAW_CONTROL",
                std::abs(waist_error_), WAIST_THRESHOLD_ENTER, lateral_cmd_);
            stage_ = FlightStage::YAW_CONTROL;
        }
    }

    void handle_yaw_control()
    {
        if (!vision_alive())
        {
            RCLCPP_INFO(
                get_logger(),
                "YAW_CONTROL: vision timeout -> HOVER");
            hover_reference_ = vehicle_odom_;
            publish_yaw_velocity(0.0);
            stage_ = FlightStage::HOVER;
            return;
        }

        if (std::abs(waist_error_) < WAIST_THRESHOLD_EXIT && lateral_cmd_ == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "YAW_CONTROL: |waist_error|=%.1f < %.1f -> HOVER",
                std::abs(waist_error_), WAIST_THRESHOLD_EXIT);
            hover_reference_ = vehicle_odom_;
            publish_yaw_velocity(0.0);
            stage_ = FlightStage::HOVER;
            return;
        }

        double dt = compute_dt();

        yaw_rate_cmd_ = -(pid_yaw_.update(waist_error_, dt)/100.0) * MAX_YAW_RATE;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "YAW_CONTROL: waist_error=%.1f dt=%.3f yaw_rate_cmd=%.3f",
            waist_error_, dt, yaw_rate_cmd_);

        publish_yaw_velocity(yaw_rate_cmd_);
    }

    void handle_land()
    {
        if (current_state_.mode != "LAND")
        {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "LAND";
            mode_client_->async_send_request(req);
        }
    }

    // ============================================================
    // ===================== PUBLISHERS ===========================
    // ============================================================

    // void publish_position_hold()
    // {
    //     geometry_msgs::msg::PoseStamped pose;

    //     pose.header.stamp = now();

    //     pose.pose.position.x = hover_reference_.position.x;
    //     pose.pose.position.y = hover_reference_.position.y;
    //     pose.pose.position.z = hover_reference_.position.z;

    //     // Also hold the current yaw (full orientation) at the hover reference.
    //     pose.pose.orientation.x = hover_reference_.quaternion.x;
    //     pose.pose.orientation.y = hover_reference_.quaternion.y;
    //     pose.pose.orientation.z = hover_reference_.quaternion.z;
    //     pose.pose.orientation.w = hover_reference_.quaternion.w;

    //     pos_pub_->publish(pose);
    // }

    void publish_stop_velocity()
    {
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = now();
        vel_pub_->publish(twist);
    }

    void publish_yaw_velocity(double yaw_cmd)
    {
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = now();

        // 1. ALTITUDE HOLD (Z-Axis)
        double z_error = hover_reference_.position.z - vehicle_odom_.position.z;
        double Kp_z = 1.5; 
        twist.twist.linear.z = Kp_z * z_error;

        // 2. LATERAL MOVEMENT & HORIZONTAL HOLD
        double Kp_xy = 1.0; 

        if (lateral_cmd_ != 0) 
        {
            // We want to move left or right relative to the drone's body
            double v_xb = 0.0; // Forward/Backward velocity (keep 0 for now)
            
            // In ROS ENU, Left is Positive Y, Right is Negative Y
            double v_yb = (lateral_cmd_ == 1) ? LATERAL_SPEED : -LATERAL_SPEED; 
            
	    

            // Rotate body-frame velocity into the global ENU frame using the drone's current yaw
            double yaw = vehicle_odom_.orientation.yaw;
            twist.twist.linear.x = v_xb * std::cos(yaw) - v_yb * std::sin(yaw);
            twist.twist.linear.y = v_xb * std::sin(yaw) + v_yb * std::cos(yaw);

            // CRITICAL: Continuously update the hover reference while moving
            // so it locks into its NEW position when you stop!
            hover_reference_.position.x = vehicle_odom_.position.x;
            hover_reference_.position.y = vehicle_odom_.position.y;
        }
        else 
        {
            // Apply Position Hold to stay exactly where we are
            double x_error = hover_reference_.position.x - vehicle_odom_.position.x;
            double y_error = hover_reference_.position.y - vehicle_odom_.position.y;
            twist.twist.linear.x = Kp_xy * x_error;
            twist.twist.linear.y = Kp_xy * y_error;
        }

        // 3. YAW CONTROL (Spinning)
        twist.twist.angular.z = yaw_cmd;
        twist.twist.angular.x = 0.0;
        twist.twist.angular.y = 0.0;

        vel_pub_->publish(twist);
    }
};

// ================================================================
// ============================ MAIN ==============================
// ================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<HumanTrackingNode>());

    rclcpp::shutdown();

    return 0;
}
