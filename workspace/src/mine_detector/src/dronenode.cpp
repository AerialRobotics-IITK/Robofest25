#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

struct DroneLog {
  float x;
  float y;
  float timestamp;
};

class DroneNode : public rclcpp::Node {
public:
  DroneNode() : Node("drone_node") {
    namespace_ = declare_parameter<std::string>("namespace", "/uav1");
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 1.5);
    forward_distance_ = declare_parameter<double>("forward_distance", 3.0);
    forward_hold_sec_ = declare_parameter<double>("forward_hold_sec", 8.0);
    land_mode_ = declare_parameter<std::string>("land_mode", "LAND");

    init_csv();

    mission_start_time_ = std::chrono::steady_clock::now();

    sub_ = create_subscription<std_msgs::msg::Float64>(
        "/timestamp_pipeline", 10,
        std::bind(&DroneNode::pipeline_callback, this, std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        namespace_ + "/state", rclcpp::QoS(10).best_effort(),
        std::bind(&DroneNode::mavros_state_callback, this,
                  std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        namespace_ + "/local_position/pose", rclcpp::QoS(10).best_effort(),
        std::bind(&DroneNode::pose_callback, this, std::placeholders::_1));

    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        namespace_ + "/setpoint_position/local", rclcpp::QoS(10).best_effort());

    arm_client_ = create_client<mavros_msgs::srv::CommandBool>(
        namespace_ + "/cmd/arming");
    mode_client_ =
        create_client<mavros_msgs::srv::SetMode>(namespace_ + "/set_mode");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(
        namespace_ + "/cmd/takeoff");

    flush_timer_ =
        create_wall_timer(30s, std::bind(&DroneNode::flush_old_array, this));
    command_timer_ =
        create_wall_timer(100ms, std::bind(&DroneNode::command_loop, this));
  }

  ~DroneNode() override {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  enum class FlightStage { WAIT_INIT, TAKEOFF, FORWARD, LANDING, COMPLETE };

  std::string namespace_;
  double takeoff_altitude_{1.5};
  double forward_distance_{3.0};
  double forward_hold_sec_{8.0};
  std::string land_mode_{"LAND"};

  FlightStage stage_{FlightStage::WAIT_INIT};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::vector<DroneLog> logs_;
  std::vector<DroneLog> detected_mines_;
  std::ofstream csv_file_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_pose_{false};
  bool takeoff_requested_{false};
  bool land_requested_{false};

  double hover_x_{0.0};
  double hover_y_{0.0};
  double hover_z_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};

  std::chrono::steady_clock::time_point mission_start_time_;
  std::chrono::steady_clock::time_point forward_stage_start_time_;

  void init_csv() {
    const std::string folder_name = "mines_detected";
    std::filesystem::create_directories(folder_name);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char time_buffer[64];
    std::strftime(time_buffer, sizeof(time_buffer), "%d_%H%M%S", &tm);

    const std::string filename =
        folder_name + "/mine@" + std::string(time_buffer) + ".csv";

    csv_file_.open(filename, std::ios::out | std::ios::app);
    if (csv_file_.is_open()) {
      csv_file_ << "x,y,timestamp\n";
      RCLCPP_INFO(get_logger(), "Logging detections to: %s", filename.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file for writing.");
    }
  }

  void mavros_state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "State: connected=%d armed=%d mode=%s",
                         current_state_.connected, current_state_.armed,
                         current_state_.mode.c_str());
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    has_pose_ = true;

    const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      mission_start_time_)
            .count();

    logs_.push_back({static_cast<float>(msg->pose.position.x),
                     static_cast<float>(msg->pose.position.y),
                     static_cast<float>(elapsed)});

    if (logs_.size() > 5000) {
      logs_.erase(logs_.begin(), logs_.begin() + 1000);
    }
  }

  void flush_old_array() {
    if (!logs_.empty()) {
      logs_.clear();
      RCLCPP_INFO(get_logger(), "Flushed old drone logs array.");
    }
  }

  void pipeline_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    const double received_time = msg->data;

    if (received_time == -1.0) {
      return;
    }

    if (logs_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Drone log array is empty. Cannot match timestamp %.2f.",
                  received_time);
      return;
    }

    const auto it = std::lower_bound(
        logs_.begin(), logs_.end(), received_time,
        [](const DroneLog &log, double value) { return log.timestamp < value; });

    DroneLog closest_log;
    if (it == logs_.begin()) {
      closest_log = *it;
    } else if (it == logs_.end()) {
      closest_log = *(it - 1);
    } else {
      const auto prev = it - 1;
      closest_log =
          (std::abs(received_time - prev->timestamp) <
           std::abs(received_time - it->timestamp))
              ? *prev
              : *it;
    }

    detected_mines_.push_back(closest_log);
    RCLCPP_INFO(get_logger(), "MINE DETECTED @ (%.2f, %.2f)", closest_log.x,
                closest_log.y);

    if (csv_file_.is_open()) {
      csv_file_ << closest_log.x << "," << closest_log.y << ","
                << closest_log.timestamp << "\n";
      csv_file_.flush();
    }

    flush_old_array();
    flush_timer_->reset();
  }

  void command_loop() {
    switch (stage_) {
    case FlightStage::WAIT_INIT:
      handle_wait_init();
      break;
    case FlightStage::TAKEOFF:
      handle_takeoff();
      break;
    case FlightStage::FORWARD:
      handle_forward();
      break;
    case FlightStage::LANDING:
      handle_landing();
      break;
    case FlightStage::COMPLETE:
      break;
    }
  }

  void handle_wait_init() {
    if (!current_state_.connected || !has_pose_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "WAIT_INIT: Waiting for MAVROS (connected=%d has_pose=%d)",
          current_state_.connected, has_pose_);
      return;
    }

    if (!mode_client_->service_is_ready() || !arm_client_->service_is_ready() ||
        !takeoff_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "WAIT_INIT: Waiting for MAVROS services.");
      return;
    }

    if (current_state_.mode != "GUIDED") {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "GUIDED";
      mode_client_->async_send_request(req);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "SET_MODE: requesting GUIDED");
      return;
    }

    if (!current_state_.armed) {
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      arm_client_->async_send_request(req);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "ARM: sending arm command");
      return;
    }

    if (!takeoff_requested_) {
      auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude = takeoff_altitude_;
      takeoff_client_->async_send_request(req);
      takeoff_requested_ = true;
      stage_ = FlightStage::TAKEOFF;
      RCLCPP_INFO(get_logger(), "TAKEOFF: requesting altitude %.2f m",
                  takeoff_altitude_);
    }
  }

  void handle_takeoff() {
    const double z = current_pose_.pose.position.z;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "TAKEOFF: z=%.2f (target=%.2f)", z,
                         takeoff_altitude_);

    if (z >= takeoff_altitude_ - 0.15) {
      hover_x_ = current_pose_.pose.position.x;
      hover_y_ = current_pose_.pose.position.y;
      hover_z_ = current_pose_.pose.position.z;

      target_x_ = hover_x_ + forward_distance_;
      target_y_ = hover_y_;
      target_z_ = takeoff_altitude_;
      forward_stage_start_time_ = std::chrono::steady_clock::now();
      stage_ = FlightStage::FORWARD;

      RCLCPP_INFO(get_logger(),
                  "TAKEOFF: reached altitude. Moving forward to x=%.2f y=%.2f "
                  "z=%.2f",
                  target_x_, target_y_, target_z_);
    }
  }

  void handle_forward() {
    publish_position_setpoint(target_x_, target_y_, target_z_);

    const double dx = target_x_ - current_pose_.pose.position.x;
    const double dy = target_y_ - current_pose_.pose.position.y;
    const double dz = target_z_ - current_pose_.pose.position.z;
    const double distance_to_target = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      forward_stage_start_time_)
            .count();

    if (distance_to_target < 0.25 || elapsed >= forward_hold_sec_) {
      request_landing();
      stage_ = FlightStage::LANDING;
      RCLCPP_INFO(get_logger(),
                  "FORWARD: forward leg complete. Requesting landing.");
    }
  }

  void handle_landing() {
    if (!land_requested_) {
      request_landing();
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LANDING: current altitude %.2f m",
                         current_pose_.pose.position.z);

    if (has_pose_ && current_pose_.pose.position.z <= 0.15 &&
        !current_state_.armed) {
      stage_ = FlightStage::COMPLETE;
      RCLCPP_INFO(get_logger(), "Mission complete: landed and disarmed.");
    }
  }

  void request_landing() {
    if (land_requested_) {
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = land_mode_;
    mode_client_->async_send_request(req);
    land_requested_ = true;
    RCLCPP_INFO(get_logger(), "SET_MODE: requesting %s", land_mode_.c_str());
  }

  void publish_position_setpoint(double x, double y, double z) {
    geometry_msgs::msg::PoseStamped setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = "map";
    setpoint.pose.position.x = x;
    setpoint.pose.position.y = y;
    setpoint.pose.position.z = z;
    setpoint.pose.orientation.w = 1.0;
    pos_pub_->publish(setpoint);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
