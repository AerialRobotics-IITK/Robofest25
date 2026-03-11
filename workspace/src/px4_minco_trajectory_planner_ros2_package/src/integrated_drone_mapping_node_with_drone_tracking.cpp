#include <rclcpp/rclcpp.hpp>

// ================ Message Imports =================
// Standard ROS messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// MAVROS interfaces (new control + state backend)
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ================= Mapping & SFC =================
#include "px4_minco_trajectory_planner_ros2_package/SFC_generation/corridor_builder.hpp"
#include "px4_minco_trajectory_planner_ros2_package/SFC_generation/final_obstacle_inflate.hpp"
#include "px4_minco_trajectory_planner_ros2_package/mapping/grid_mapper.hpp"

// ================= GCOPTER =================
#include "px4_minco_trajectory_planner_ros2_package/minco_planner/gcopter.hpp"
#include "px4_minco_trajectory_planner_ros2_package/minco_planner/trajectory.hpp"

// ================= Shared Utils (vehicle odometry helpers) =================
#include "human_tracking_controls/Utils/odometry_utils.hpp"
#include "human_tracking_controls/Utils/vehicle_utils.hpp"

using namespace Eigen;
using namespace vehicle_utils;
using namespace std::chrono_literals;

class IntegratedMappingSFCGCOPTER : public rclcpp::Node {

public:
  IntegratedMappingSFCGCOPTER() : Node("integrated_mapping_sfc_gcopter") {

    namespace_ = declare_parameter<std::string>("namespace", "/uav1");

    RCLCPP_INFO(get_logger(), "🚀 Mapping → SFC → GCOPTER (Reactive)");

    init_swarm_params();
    init_mapper();
    init_sfc();
    init_ros();
    init_timer();
  }

private:
  std::string namespace_;
  rclcpp::QoS pub_qos_ = rclcpp::QoS(10).best_effort();
  rclcpp::QoS mavros_qos_ = rclcpp::QoS(10).best_effort();

  // ============================================================
  // ===================== FLIGHT STAGES ========================
  // ============================================================

  enum class FlightStage { WAIT_INIT, TAKEOFF, HOVER, TRACKING };

  FlightStage stage_ = FlightStage::WAIT_INIT;

  // THIS IS JUST FOR LOGGING PURPOSES

  enum class TrackingState {
    NO_ODOM,
    NO_TRAJECTORY,
    TRACKING,
    TRAJECTORY_FINISHED,
    GOAL_REACHED
  };

  TrackingState tracking_state_ = TrackingState::NO_TRAJECTORY;
  TrackingState last_tracking_state_ = TrackingState::NO_TRAJECTORY;

  // ============================================================
  // ===================== CONSTANTS ============================
  // ============================================================

  static constexpr float TAKEOFF_ALTITUDE = 1.0f;

  // ============================================================
  // ===================== TRAJECTORY DATA ========================
  // ============================================================

  struct TrajData {
    double t;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
  };

  std::vector<IntegratedMappingSFCGCOPTER::TrajData> global_traj_;

  int global_traj_index_ = 0;

  /* ================= MAP PARAMS ================= */

  static constexpr float GRID_W = 50.0f;
  static constexpr float GRID_H = 50.0f;
  static constexpr float RES = 0.05f;

  /* ================= RUNTIME PARAMS ================= */
  double goal_x_{20.0};
  double goal_y_{-9.0};
  double goal_z_{1.0};
  double drone_radius_cm_{35.0};
  int lookahead_indices_{200};
  double max_vel_{0.5};
  double pipeline_step_duration_{0.2};

  /* ================= GRID ================= */

  int grid_w_ = GRID_W / RES;
  int grid_h_ = GRID_H / RES;
  int replan_id_{0};
  bool replanning_{false};
  /* ================= STATE ================= */

  double x_{0}, y_{0}, z_{1.0}, yaw_{0};
  double x_ned_{0}, y_ned_{0}, z_ned_{-1.0}, yaw_ned_{0.0}; // IN NED FRAME

  bool has_odom_{false};
  bool has_scan_{false};

  VehicleOdometry vehicle_odom_;
  mavros_msgs::msg::State current_state_;

  /* ================= DATA ================= */

  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::OccupancyGrid occ_msg_;
  std::vector<int8_t> occ_data_;
  std::vector<uint8_t> bin_map_;
  std::vector<Eigen::Vector3d> executed_path_;

  /* ================= MODULES ================= */

  std::shared_ptr<OccupancyMapper> mapper_;
  std::shared_ptr<CorridorBuilder> corridor_;

  /* ================= TRAJECTORY ================= */

  Trajectory<3> current_traj_;
  bool has_traj_{false};

  /* ================= ROS ================= */

  // Hover reference in ENU
  double hover_x_ = 0.0;
  double hover_y_ = 0.0;
  double hover_z_ = 0.0;

  // MAVROS publishers (velocity / position + path)
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tracking_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr self_traj_path_pub_;

  // MAVROS subscriptions
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr>
      peer_traj_subs_;

  // MAVROS services
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::QoS qos_profile = rclcpp::QoS(10).best_effort().durability_volatile();

  rclcpp::TimerBase::SharedPtr pipeline_step_timer_;
  rclcpp::TimerBase::SharedPtr command_loop_timer_;

  int counter_ = 0;

  // int width = 100;

  struct PeerTrajRecord {
    Trajectory<3> traj;
    rclcpp::Time recv_time;
    bool valid{false};
  };

  bool swarm_enabled_{false};
  double swarm_threshold_{1.0};
  std::vector<double> swarm_ellipsoid_diag_{1.0, 1.0, 1.0};
  double swarm_peer_timeout_sec_{0.3};
  std::vector<std::string> swarm_peer_topics_;
  std::string swarm_self_trajectory_topic_;
  double swarm_default_segment_dt_{0.1};
  std::unordered_map<std::string, PeerTrajRecord> peer_trajs_;

  /* ========================================================= */
  /* ================= INIT ================================ */
  /* ========================================================= */
  void init_swarm_params() {
    swarm_enabled_ = declare_parameter<bool>("swarm.enabled", false);
    swarm_threshold_ = declare_parameter<double>("swarm.threshold", 1.0);
    swarm_ellipsoid_diag_ = declare_parameter<std::vector<double>>(
        "swarm.ellipsoid_diag", {1.0, 1.0, 1.0});
    swarm_peer_timeout_sec_ =
        declare_parameter<double>("swarm.peer_timeout_sec", 0.3);
    swarm_peer_topics_ = declare_parameter<std::vector<std::string>>(
        "swarm.peer_topics", std::vector<std::string>{});
    swarm_self_trajectory_topic_ = declare_parameter<std::string>(
        "swarm.self_trajectory_topic", "/swarm/self_trajectory");

    if (swarm_ellipsoid_diag_.size() != 3) {
      RCLCPP_WARN(get_logger(), "swarm.ellipsoid_diag must have 3 values. "
                                "Falling back to [1, 1, 1].");
      swarm_ellipsoid_diag_ = {1.0, 1.0, 1.0};
    }

    if (swarm_peer_timeout_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "swarm.peer_timeout_sec must be > 0. Falling back to 0.3s.");
      swarm_peer_timeout_sec_ = 0.3;
    }

    // Load newly added dynamic params
    goal_x_ = declare_parameter("goal_x", 20.0);
    goal_y_ = declare_parameter("goal_y", -9.0);
    goal_z_ = declare_parameter("goal_z", 1.0);
    drone_radius_cm_ = declare_parameter("drone_radius_cm", 35.0);
    max_vel_ = declare_parameter("max_vel", 0.5);
    lookahead_indices_ = declare_parameter("lookahead_indices", 200);
    pipeline_step_duration_ = declare_parameter("pipeline_step_duration", 0.2);
  }

  Trajectory<3> path_to_trajectory(const nav_msgs::msg::Path &path) const {
    Trajectory<3> traj;
    const size_t n = path.poses.size();
    if (n < 2) {
      return traj;
    }

    for (size_t i = 0; i + 1 < n; ++i) {
      const auto &p0_msg = path.poses[i];
      const auto &p1_msg = path.poses[i + 1];

      const Eigen::Vector3d p0(p0_msg.pose.position.x, p0_msg.pose.position.y,
                               p0_msg.pose.position.z);
      const Eigen::Vector3d p1(p1_msg.pose.position.x, p1_msg.pose.position.y,
                               p1_msg.pose.position.z);

      double dt = swarm_default_segment_dt_;
      const double t0 = rclcpp::Time(p0_msg.header.stamp).seconds();
      const double t1 = rclcpp::Time(p1_msg.header.stamp).seconds();
      if (t1 > t0 + 1.0e-3) {
        dt = t1 - t0;
      }

      if (dt < 1.0e-3) {
        dt = swarm_default_segment_dt_;
      }

      Piece<3>::CoefficientMat coeff;
      coeff.setZero();
      coeff.col(3) = p0;
      coeff.col(2) = (p1 - p0) / dt;
      traj.emplace_back(dt, coeff);
    }

    return traj;
  }

  void peer_path_cb(const nav_msgs::msg::Path::SharedPtr msg,
                    const std::string &topic) {
    Trajectory<3> traj = path_to_trajectory(*msg);
    auto &record = peer_trajs_[topic];
    record.recv_time = now();
    record.valid = traj.getPieceNum() > 0;
    record.traj = traj;
  }

  std::vector<Trajectory<3>> collect_fresh_peer_trajectories() const {
    std::vector<Trajectory<3>> other_agents;
    const rclcpp::Time t_now = now();
    for (const auto &it : peer_trajs_) {
      const auto &record = it.second;
      if (!record.valid) {
        continue;
      }

      const double age = (t_now - record.recv_time).seconds();
      if (age <= swarm_peer_timeout_sec_) {
        other_agents.push_back(record.traj);
      }
    }
    return other_agents;
  }

  void init_peer_traj_subscribers() {
    peer_traj_subs_.clear();
    peer_trajs_.clear();

    if (!swarm_enabled_) {
      RCLCPP_INFO(get_logger(), "Swarm avoidance disabled by parameter.");
      return;
    }

    if (swarm_peer_topics_.empty()) {
      RCLCPP_WARN(get_logger(), "swarm.enabled=true but swarm.peer_topics is "
                                "empty. Swarm penalty will be inactive.");
      return;
    }

    for (const auto &topic : swarm_peer_topics_) {
      auto sub = create_subscription<nav_msgs::msg::Path>(
          topic, rclcpp::QoS(10).best_effort(),
          [this, topic](const nav_msgs::msg::Path::SharedPtr msg) {
            this->peer_path_cb(msg, topic);
          });
      peer_traj_subs_.push_back(sub);
      peer_trajs_.emplace(topic, PeerTrajRecord());
    }

    RCLCPP_INFO(
        get_logger(),
        "Swarm avoidance enabled. Subscribed to %zu peer trajectory topics.",
        peer_traj_subs_.size());
  }

  nav_msgs::msg::Path trajectory_to_path_msg(const Trajectory<3> &traj) const {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    if (traj.getPieceNum() == 0) {
      return path_msg;
    }

    double t_global = 0.0;
    const rclcpp::Time t0 = now();
    for (int i = 0; i < traj.getPieceNum(); ++i) {
      const auto &piece = traj[i];
      const auto &c = piece.getCoeffMat();
      const double dur = piece.getDuration();
      const double sample_dt = 0.05;

      for (double t = 0.0; t < dur; t += sample_dt) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = path_msg.header.frame_id;
        const rclcpp::Time stamp_time =
            t0 + rclcpp::Duration::from_seconds(t_global);
        const int64_t stamp_ns = stamp_time.nanoseconds();
        ps.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        ps.header.stamp.nanosec =
            static_cast<uint32_t>(stamp_ns % 1000000000LL);

        Eigen::Vector3d p;
        const double t2 = t * t;
        const double t3 = t2 * t;
        for (int k = 0; k < 3; ++k) {
          p(k) = c(k, 3) + c(k, 2) * t + c(k, 1) * t2 + c(k, 0) * t3;
        }
        ps.pose.position.x = p.x();
        ps.pose.position.y = p.y();
        ps.pose.position.z = p.z();
        ps.pose.orientation.w = 1.0;

        path_msg.poses.push_back(ps);
        t_global += sample_dt;
      }
    }

    return path_msg;
  }

  void init_mapper() {

    mapper_ = std::make_shared<OccupancyMapper>(GRID_W, GRID_H, RES, 20.0);
  }

  void init_sfc() {

    corridor_ = std::make_shared<CorridorBuilder>(grid_w_, grid_h_, RES,
                                                  RES * 100.0, drone_radius_cm_);
  }

  void init_ros() {
    // ----------------- Publishers -----------------
    // MAVROS publishers
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        namespace_ + "/setpoint_velocity/cmd_vel", pub_qos_);

    tracking_error_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        namespace_ + "/tracking_error", pub_qos_);

    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        namespace_ + "/setpoint_position/local", pub_qos_);

    self_traj_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        swarm_self_trajectory_topic_, rclcpp::QoS(10).best_effort());

    // ----------------- Subscribers -----------------
    // MAVROS state + odometry
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        namespace_ + "/state", mavros_qos_,
        std::bind(&IntegratedMappingSFCGCOPTER::mavros_state_cb, this,
                  std::placeholders::_1));

    pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        namespace_ + "/local_position/pose", mavros_qos_,
        std::bind(&IntegratedMappingSFCGCOPTER::pose_cb, this,
                  std::placeholders::_1));

    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        namespace_ + "/local_position/velocity_local", mavros_qos_,
        std::bind(&IntegratedMappingSFCGCOPTER::vel_cb, this,
                  std::placeholders::_1));

    // Common scan subscription
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/uav1/scan", rclcpp::QoS(10).best_effort(),
        std::bind(&IntegratedMappingSFCGCOPTER::scan_cb, this,
                  std::placeholders::_1));

    init_peer_traj_subscribers();

    // ----------------- MAVROS services -----------------
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>(namespace_ +
                                                               "/cmd/arming");
    mode_client_ =
        create_client<mavros_msgs::srv::SetMode>(namespace_ + "/set_mode");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(
        namespace_ + "/cmd/takeoff");

    arm_client_->wait_for_service();
    mode_client_->wait_for_service();
    takeoff_client_->wait_for_service();
  }

  void init_timer() {

    pipeline_step_timer_ = create_wall_timer(
        std::chrono::duration<double>(pipeline_step_duration_), std::bind(&IntegratedMappingSFCGCOPTER::pipeline_step, this));

    command_loop_timer_ = create_wall_timer(
        0.01s, std::bind(&IntegratedMappingSFCGCOPTER::command_loop, this));
  }

  /* ========================================================= */
  /* ================= CALLBACKS ============================= */
  /* ========================================================= */

  // ===== MAVROS-based state + odometry callbacks (ENU) =====
  void mavros_state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "State: connected=%d armed=%d mode=%s",
                         current_state_.connected, current_state_.armed,
                         current_state_.mode.c_str());
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    vehicle_odom_.position.x = msg->pose.position.x;
    vehicle_odom_.position.y = msg->pose.position.y;
    vehicle_odom_.position.z = msg->pose.position.z;

    vehicle_odom_.quaternion.x = msg->pose.orientation.x;
    vehicle_odom_.quaternion.y = msg->pose.orientation.y;
    vehicle_odom_.quaternion.z = msg->pose.orientation.z;
    vehicle_odom_.quaternion.w = msg->pose.orientation.w;

    odom_utils::quaternionToEuler(vehicle_odom_.quaternion,
                                  vehicle_odom_.orientation);

    // Feed mapping pose (ENU)
    x_ = vehicle_odom_.position.x;
    y_ = vehicle_odom_.position.y;
    z_ = vehicle_odom_.position.z;
    yaw_ = vehicle_odom_.orientation.yaw;

    has_odom_ = true;
    executed_path_.push_back(
    Eigen::Vector3d(
        vehicle_odom_.position.x,
        vehicle_odom_.position.y,
        vehicle_odom_.position.z));
  }

  void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    vehicle_odom_.velocity.x = msg->twist.linear.x;
    vehicle_odom_.velocity.y = msg->twist.linear.y;
    vehicle_odom_.velocity.z = msg->twist.linear.z;
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    last_scan_ = *msg;
    has_scan_ = true;
  }

  /* ========================================================= */
  /* ================= MAPPING ============================== */
  /* ========================================================= */

  void update_map_step() {
    if (!has_odom_ || !has_scan_)
      return;

    /* ===== UPDATE MAP ===== */

    // std::cout << "yaw = " << yaw_ << std::endl;
    // std::cout << "x = " << x_ << std::endl;
    // std::cout << "y = " << y_ << std::endl;

    mapper_->update_map( // everything must be ENU
        x_, y_, yaw_, last_scan_.ranges, last_scan_.angle_min,
        last_scan_.angle_increment);

    occ_msg_ = mapper_->get_grid_message("map", now());

    occ_data_ = occ_msg_.data;

    /* ===== BUILD MAP ===== */

    auto raw_bin = build_binary_map();

    bin_map_ = ObstacleInflator::inflateObstacles(raw_bin, grid_w_, grid_h_,
                                                  RES, drone_radius_cm_);

    // std::ofstream file("binary_map_log.txt"); // overwrite mode

    // if (!file.is_open()) {
    //     RCLCPP_ERROR(get_logger(), "Failed to open binary_map_log.txt");
    // } else {

    //     file << "[\n";

    //     for (int y = 0; y < grid_h_; ++y) {
    //         file << "  ["; // start row

    //         for (int x = 0; x < grid_w_; ++x) {
    //             int idx = y * grid_w_ + x;
    //             // cast to int so uint8_t prints as number
    //             file << static_cast<int>(bin_map_[idx]);

    //             if (x != grid_w_ - 1) {
    //                 file << ",";      // comma between elements
    //                 // optional space for readability:
    //                 file << " ";
    //             }
    //         }

    //         file << "]";

    //         if (y != grid_h_ - 1) {
    //             file << ",";   // comma between rows
    //         }

    //         file << "\n";
    //     }

    //     file << "]\n";

    //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "bin_map_
    //     logged to binary_map_log.txt in map_data format.");

    /*

    std::ofstream file("binary_map_log.txt");

        if (!file.is_open()) {
            std::cerr << "Failed to open file!\n";
            return;
        }

        for (size_t i = 0; i < bin_map_.size(); ++i)
        {
            // Convert uint8_t to int to avoid ASCII character printing
            file << static_cast<int>(bin_map_[i]) << ", ";

            // New line after each row (if it's a 2D grid stored row-major)
            if ((i + 1) % width == 0)
                file << "\n  ";
        }

        file.close();

    */
  }

  /* ========================================================= */
  /* ================= PIPELINE ============================== */
  /* ========================================================= */

  void pipeline_step() {

    if (!has_odom_ || !has_scan_)
      return;

    update_map_step();

    save_map(); 

    /* ===== HEAD / TAIL ===== */

    Matrix3d head, tail;

    generate_head_tail(head, tail);

    /* ===== SAFETY CHECK ===== */

    bool need_replan = true;

    if (has_traj_) {

      need_replan = !trajectory_is_safe(global_traj_,
                                       lookahead_indices_, // lookahead index
                                        0.05, bin_map_);
    }

    if (!need_replan)
      return;

    replanning_ = true;
    publish_stop_velocity();

    RCLCPP_WARN(get_logger(), "🔄 Replanning...");

    // global_traj_index_=0;                       // again come back to
    // starting index

    /* ===== BUILD CORRIDOR ===== */

    Vector2i start = world_to_grid(head(0, 0), head(1, 0));

    Vector2i goal = world_to_grid(tail(0, 0), tail(1, 0));

    if (!corridor_->build(bin_map_, start.x(), start.y(), goal.x(), goal.y())) {

      RCLCPP_WARN(get_logger(), "SFC failed");

      return;
    }

    auto sfc2d = corridor_->getInequalities();

    if (sfc2d.empty())
      return;

    /* ===== LIFT TO 3D ===== */

    std::vector<MatrixX4d> corridor3d;

    double z_min = z_ - 0.5;
    double z_max = z_ + 2.0;

    /*for (auto &poly : sfc2d)
    {

        corridor3d.push_back(
            gridSFCToWorld(
                poly.first,
                poly.second,
                z_min,
                z_max));
    }*/

    for (size_t i = 0; i < sfc2d.size(); ++i) {
      auto &poly = sfc2d[i];
      // use poly here

      corridor3d.push_back(
          gridSFCToWorld(poly.first, poly.second, z_min, z_max));
    }

    // ==========================================
    // LOG SFC DATA (2D + 3D)
    // ==========================================

    // std::ofstream file("sfc_constraints.txt");

    // if (!file.is_open()) {
    //     RCLCPP_ERROR(get_logger(), "Failed to open sfc_log.txt");
    // } else {

    //     // -------- Timestamp --------
    //     std::time_t now = std::time(nullptr);
    //     file << "\n\n==================================================\n";
    //     file << "SFC LOG ENTRY\n";
    //     file << "Timestamp: " << std::ctime(&now);
    //     file << "==================================================\n";

    //     // ======================================================
    //     // 1️⃣ 2D INEQUALITIES
    //     // ======================================================
    //     file << "\n===== 2D Safe Flight Corridor (A x <= b) =====\n";

    //     for (size_t i = 0; i < sfc2d.size(); ++i)
    //     {
    //         file << "\n--- Polytope " << i << " ---\n";

    //         const Eigen::MatrixXd& A = sfc2d[i].first;
    //         const Eigen::VectorXd& b = sfc2d[i].second;

    //         file << "Number of constraints: " << A.rows() << "\n";

    //         for (int row = 0; row < A.rows(); ++row)
    //         {
    //             file << "Constraint " << row << ": ";

    //             file << A(row, 0) << " * x + "
    //                 << A(row, 1) << " * y <= "
    //                 << b(row) << "\n";
    //         }
    //     }

    //     file.close();

    //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "SFC
    //     successfully logged.");

    /* ===== GCOPTER ===== */

    run_gcopter(corridor3d, head, tail);
  }

  /* ========================================================= */
  /* ================= SAFETY ================================ */
  /* ========================================================= */

  bool trajectory_is_safe(
      const std::vector<IntegratedMappingSFCGCOPTER::TrajData> &traj,
      int lookahead, double dt, const std::vector<uint8_t> &map) {

    double index = global_traj_index_;

    while (index < (lookahead + global_traj_index_) &&
          index < traj.size()) {

      Vector3d p = traj[index].p;

      if (is_occupied(p.x(), p.y(), grid_w_, map)) {

        RCLCPP_WARN(get_logger(), "⚠️ Obstacle at %.2f %.2f", p.x(), p.y());

        return false;
      }

      index += 1;
    }

    return true;
  }

  /* ========================================================= */
  /* ================= HEAD / TAIL ============================ */
  /* ========================================================= */

  // void generate_head_tail(Vector3d &head, Vector3d &tail)
  // {

  //     // Start = current drone pose
  //     head << vehicle_odom_.position.x, vehicle_odom_.position.y, GOAL_Z;

  //     // Goal = fixed user-defined target
  //     tail << GOAL_X, GOAL_Y, GOAL_Z;
  // }

  void generate_head_tail(Matrix3d &head, Matrix3d &tail) {
    head.setZero();
    tail.setZero();

    // Start State: Current position AND Current Velocity
    head.col(0) << vehicle_odom_.position.x, vehicle_odom_.position.y,
        vehicle_odom_.position.z;
    head.col(1) << vehicle_odom_.velocity.x, vehicle_odom_.velocity.y,
        vehicle_odom_.velocity.z;
    // Acceleration (col 2) stays 0 unless you have filtered IMU data

    // Goal State: Desired position, stopping at rest (v=0, a=0)
    tail.col(0) << goal_x_, goal_y_, goal_z_;
  }

  /* ========================================================= */
  /* ================= GCOPTER ================================ */
  /* ========================================================= */

  void run_gcopter(const std::vector<MatrixX4d> &corridor, const Matrix3d &head,
                   const Matrix3d &tail) {

    gcopter::GCOPTER_PolytopeSFC sfc;

    // Matrix3d head, tail;

    // head.setZero();
    // tail.setZero();

    // head.col(0) = head_pos;
    // tail.col(0) = tail_pos;

    VectorXd bounds(6), weights(6), params(6);

    bounds << 0.5, 4, 1, -1, 2, 4;
    weights << 3.0, 3.0, 1, 0.05, 0.05, 1;
    params << 0.5, 9.81, 0, 0, 0, 1;

    // vmax, w_dot_max , tilt_max , thrust_min , thrust_max , r_min

    // mass, gravity, drag_h, drag_v, drag_p, speed_smooth

    if (!sfc.setup(0.7, head, tail, corridor, 0.25, 0.08, 4, bounds, weights,
                   params)) {

      RCLCPP_ERROR(get_logger(), "GCOPTER setup failed");

      return;
    }

    if (swarm_enabled_) {
      std::vector<Trajectory<3>> other_agents =
          collect_fresh_peer_trajectories();
      Eigen::Matrix3d E = Eigen::Matrix3d::Zero();
      E(0, 0) = swarm_ellipsoid_diag_[0];
      E(1, 1) = swarm_ellipsoid_diag_[1];
      E(2, 2) = swarm_ellipsoid_diag_[2];

      sfc.setSwarmObstacleParams(other_agents, swarm_threshold_, E);
      RCLCPP_INFO(get_logger(),
                  "Swarm penalty active: %zu/%zu peer trajectories are fresh.",
                  other_agents.size(), peer_trajs_.size());
      if (other_agents.empty()) {
        RCLCPP_WARN(get_logger(),
                    "Swarm penalty enabled but no fresh peer trajectories. "
                    "Planning without swarm repulsion.");
      }
    }

    Trajectory<3> traj;

    double cost = sfc.optimize(traj, 1e-3);

    if (traj.getPieceNum() == 0) {

      RCLCPP_ERROR(get_logger(), "GCOPTER failed");

      return;
    }

    current_traj_ = traj;
    has_traj_ = true;

    RCLCPP_WARN(get_logger(), "Saving trajectory now");

    // save_traj(traj);

    RCLCPP_WARN(get_logger(), "Updating the new trajectory planner");
    store_traj(traj);
    replanning_ = false;
    
    replan_id_++;
    save_planned_traj(); 


    if (self_traj_path_pub_) {
      nav_msgs::msg::Path my_path = trajectory_to_path_msg(traj);
      self_traj_path_pub_->publish(my_path);
      RCLCPP_INFO(get_logger(),
                  "Published self trajectory on %s with %zu poses.",
                  swarm_self_trajectory_topic_.c_str(), my_path.poses.size());
    }

    RCLCPP_INFO(get_logger(), "🎯 Trajectory cost %.3f", cost);
  }

  /* ========================================================= */
  /* ================= UTILITIES ============================== */
  /* ========================================================= */

  std::vector<uint8_t> build_binary_map() {

    std::vector<uint8_t> bin(occ_data_.size(), 0);

    for (size_t i = 0; i < occ_data_.size(); i++) {

      // unknown treated as free
      if (occ_data_[i] == 100)
        bin[i] = 1;
      else
        bin[i] = 0;
    }

    return bin;
  }

  bool is_occupied(double x, double y, int width,
                   const std::vector<uint8_t> &map) {

    Vector2i p = world_to_grid(x, y);

    int idx = p.y() * width + p.x();

    if (idx < 0 || static_cast<size_t>(idx) >= map.size())
      return true;

    return map[idx] == 1;
  }

  Vector2i world_to_grid(double x, double y) {

    return {int((x + GRID_W / 2) / RES), int((y + GRID_H / 2) / RES)};
  }

  Vector2d grid_to_world(int gx, int gy) {
    return {gx * RES - GRID_W / 2.0, gy * RES - GRID_H / 2.0};
  }

  MatrixX4d gridSFCToWorld(const MatrixXd &A, const VectorXd &b, double zmin,
                           double zmax) {

    double scale = 1.0 / RES;
    double ox = GRID_W / 2.0;
    double oy = GRID_H / 2.0;

    MatrixX4d H(A.rows() + 2, 4);

    for (int i = 0; i < A.rows(); i++) {

      double nx = A(i, 0) * scale;
      double ny = A(i, 1) * scale;

      double d = -b(i) + A(i, 0) * ox * scale + A(i, 1) * oy * scale;

      H.row(i) << nx, ny, 0, d;
    }

    H.row(A.rows()) << 0, 0, -1, zmin;
    H.row(A.rows() + 1) << 0, 0, 1, -zmax;

    return H;
  }

  /* ========================================================= */
  /* ================= SAVE ================================ */
  /* ========================================================= */

  void save_traj(const Trajectory<3> &traj) {

    // std::ofstream f("gcopter_traj.csv");

    // f << "t,px,py,pz,vx,vy,vz,ax,ay,az\n";

    // double dt = 0.01;
    // double T = 0;

    // for (int i = 0; i < traj.getPieceNum(); i++)
    // {

    //     auto &c = traj[i].getCoeffMat();
    //     double dur = traj[i].getDuration();

    //     double t = 0;

    //     while (t < dur)
    //     {

    //         Vector3d p, v, a;

    //         double t2 = t * t;
    //         double t3 = t2 * t;

    //         for (int k = 0; k < 3; k++)
    //         {

    //             p(k) = c(k, 3) + c(k, 2) * t +
    //                    c(k, 1) * t2 + c(k, 0) * t3;

    //             v(k) = c(k, 2) + 2 * c(k, 1) * t +
    //                    3 * c(k, 0) * t2;

    //             a(k) = 2 * c(k, 1) + 6 * c(k, 0) * t;
    //         }

    //         f << T << "," << p.x() << "," << p.y() << "," << p.z()
    //           << "," << v.x() << "," << v.y() << "," << v.z()
    //           << "," << a.x() << "," << a.y() << "," << a.z() << "\n";

    //         t += dt;
    //         T += dt;
    //     }
    // }

    // f.close();
  }

  /* ========================================================= */

  /* ========================================================= */
  /* ================= STORE TRAJECTORY ================================ */
  /* ========================================================= */

  void store_traj(const Trajectory<3> &traj) {
    global_traj_.clear();

    global_traj_index_ = 0; // soumya

    double dt = 0.01;
    double T = 0;

    for (int i = 0; i < traj.getPieceNum(); i++) {
      auto &c = traj[i].getCoeffMat();
      double dur = traj[i].getDuration();

      double t = 0;

      while (t < dur) {
        Eigen::Vector3d p, v, a;

        double t2 = t * t;
        double t3 = t2 * t;

        for (int k = 0; k < 3; k++) {
          p(k) = c(k, 3) + c(k, 2) * t + c(k, 1) * t2 + c(k, 0) * t3;

          v(k) = c(k, 2) + 2 * c(k, 1) * t + 3 * c(k, 0) * t2;

          a(k) = 2 * c(k, 1) + 6 * c(k, 0) * t;
        }

        global_traj_.push_back({T, p, v, a});

        t += dt;
        T += dt;
      }
    }
  }

  /* ========================================================= */
  /* ================= FOR SENDING COMMANDS TO DRONE
   * ============================== */
  /* ========================================================= */

  // Publish desired velocity (world frame) via MAVROS
  void publish_velocity_cmd(double vx, double vy, double vz) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = now();
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    vel_pub_->publish(twist);
  }

  void publish_stop_velocity() {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = now();
    vel_pub_->publish(twist);
  }

  void compute_error(double &error_x, double &error_y, double &error_z,
                     double &error_vx, double &error_vy, double &error_vz) {
    // 1. Safety Check: If we don't know where the drone is, we can't compute
    // error.
    if (!has_odom_) {
      error_x = error_y = error_z = 0.0;
      error_vx = error_vy = error_vz = 0.0;
      return;
    }

    double target_px, target_py, target_pz;
    double target_vx, target_vy, target_vz;

    // 2. Determine the Reference (Target) State
    if (global_traj_index_ >= 0 &&
        static_cast<size_t>(global_traj_index_) < global_traj_.size()) {
      // Case A: Trajectory is active -> Follow the point at the current index
      target_px = global_traj_[global_traj_index_].p.x();
      target_py = global_traj_[global_traj_index_].p.y();
      target_pz = global_traj_[global_traj_index_].p.z();

      target_vx = global_traj_[global_traj_index_].v.x();
      target_vy = global_traj_[global_traj_index_].v.y();
      target_vz = global_traj_[global_traj_index_].v.z();
    } else if (!global_traj_.empty()) {
      // Case B: Trajectory finished -> Target is the LAST point (Hover Mode)
      // We want position to hold steady, and velocity to be 0.
      target_px = global_traj_.back().p.x();
      target_py = global_traj_.back().p.y();
      target_pz = global_traj_.back().p.z();

      target_vx = 0.0;
      target_vy = 0.0;
      target_vz = 0.0;
    } else {
      // Case C: No trajectory loaded at all -> Do nothing
      error_x = 0;
      error_y = 0;
      error_z = 0;
      error_vx = 0;
      error_vy = 0;
      error_vz = 0;
      return;
    }

    // 3. Calculate Error: (Reference - Current)
    error_x = target_px - vehicle_odom_.position.x;
    error_y = target_py - vehicle_odom_.position.y;
    error_z = target_pz - vehicle_odom_.position.z;

    error_vx = target_vx - vehicle_odom_.velocity.x;
    error_vy = target_vy - vehicle_odom_.velocity.y;
    error_vz = target_vz - vehicle_odom_.velocity.z;
  }

  void tracking_pp_controller(double &vx, double &vy, double &vz) {
    vx = vy = vz = 0.0;

    // if (is_occupied(vehicle_odom_.position.x,
    //                 vehicle_odom_.position.y,
    //                 grid_w_, bin_map_)) {

    //     RCLCPP_ERROR(get_logger(), "INSIDE OBSTACLE — EMERGENCY STOP");

    //     publish_stop_velocity();
    //     stage_ = FlightStage::HOVER;
    //     return;
    // }

    // ------------- Safety: no odom -------------
    if (!has_odom_) {

      tracking_state_ = TrackingState::NO_ODOM;
      log_tracking_state();

      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "tracking_pp_controller(): no odometry -> switching to HOVER");

      // safe hover command (zero velocity)
      publish_stop_velocity();

      hover_x_ = vehicle_odom_.position.x;
      hover_y_ = vehicle_odom_.position.y;
      hover_z_ = vehicle_odom_.position.z;

      stage_ = FlightStage::HOVER;

      has_traj_ = false;

      return;
    }

    // ------------- Safety: no trajectory -------------
    if (global_traj_.empty()) {
      tracking_state_ = TrackingState::NO_TRAJECTORY;
      log_tracking_state();

      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "tracking_pp_controller(): trajectory empty -> switching to HOVER");

      publish_stop_velocity();

      hover_x_ = vehicle_odom_.position.x;
      hover_y_ = vehicle_odom_.position.y;
      hover_z_ = vehicle_odom_.position.z;

      has_traj_ = false;

      stage_ = FlightStage::HOVER;
      return;
    }

    // Compute errors with current index / last point fallback
    double ex, ey, ez, evx, evy, evz;
    compute_error(ex, ey, ez, evx, evy, evz);

    if (tracking_error_pub_) {
        geometry_msgs::msg::TwistStamped err_msg;
        err_msg.header.stamp = now();
        // Store velocity error in linear
        err_msg.twist.linear.x = evx;
        err_msg.twist.linear.y = evy;
        err_msg.twist.linear.z = evz;
        // Store position error in angular for convenience of plotting
        err_msg.twist.angular.x = ex;
        err_msg.twist.angular.y = ey;
        err_msg.twist.angular.z = ez;
        tracking_error_pub_->publish(err_msg);
    }

    // ------------- Trajectory finished (index beyond last) -------------
    if (global_traj_index_ < 0 ||
        static_cast<size_t>(global_traj_index_) >= global_traj_.size()) {

      tracking_state_ = TrackingState::TRAJECTORY_FINISHED;
      log_tracking_state();

      // ensure index stays in bounds for reading last element
      size_t last_idx = global_traj_.size() - 1;

      double goal_x = global_traj_[last_idx].p.x();
      double goal_y = global_traj_[last_idx].p.y();
      double goal_z = global_traj_[last_idx].p.z();

      // distance to final point
      double dx = goal_x - vehicle_odom_.position.x;
      double dy = goal_y - vehicle_odom_.position.y;
      double dz = goal_z - vehicle_odom_.position.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "tracking_pp_controller(): trajectory finished, dist->goal=%.3f",
          dist);

      // We want to hover at the last point. If already very close ->
      // GOAL_REACHED
      const double GOAL_THRESH = 0.5; // meters
      if (dist < GOAL_THRESH) {
        tracking_state_ = TrackingState::GOAL_REACHED;
        log_tracking_state();

        save_executed_path(); 

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "tracking_pp_controller(): goal reached -> switching to HOVER");

        // Hold zero velocity (hover)
        publish_stop_velocity();

        hover_x_ = vehicle_odom_.position.x;
        hover_y_ = vehicle_odom_.position.y;
        hover_z_ = vehicle_odom_.position.z;

        stage_ = FlightStage::HOVER;
        return;

      } else {
        // Trajectory finished but not yet at goal -> hover at last trajectory
        // point
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tracking_pp_controller(): holding last "
                             "trajectory point -> switching to HOVER");

        tracking_state_ = TrackingState::TRAJECTORY_FINISHED;
        log_tracking_state();

        hover_x_ = vehicle_odom_.position.x;
        hover_y_ = vehicle_odom_.position.y;
        hover_z_ = vehicle_odom_.position.z;

        stage_ = FlightStage::HOVER;
        has_traj_ = false;
        return;
      }
    }

    // ------------- Normal tracking case -------------
    tracking_state_ = TrackingState::TRACKING;
    log_tracking_state();

    // PD feedback in position/velocity space mapped to desired velocity
    const double KP_POS = 0.8;
    const double KV_VEL = 0.35;
    const double KA_ACC = 0.15;

    double ref_vx = 0.0, ref_vy = 0.0, ref_ax = 0.0, ref_ay = 0.0;
    if (global_traj_index_ >= 0 &&
        static_cast<size_t>(global_traj_index_) < global_traj_.size()) {
      
         ref_vx = global_traj_[global_traj_index_].v.x();
         ref_vy = global_traj_[global_traj_index_].v.y();

         ref_ax = global_traj_[global_traj_index_].a.x();
         ref_ay = global_traj_[global_traj_index_].a.y();
    }

    double cmd_vx = ref_vx + KP_POS * ex + KV_VEL * evx + KA_ACC*ref_ax;
    double cmd_vy = ref_vy + KP_POS * ey + KV_VEL * evy + KA_ACC*ref_ay;

    cmd_vx = std::clamp(cmd_vx, -max_vel_, max_vel_);
    cmd_vy = std::clamp(cmd_vy, -max_vel_, max_vel_);

    vx = cmd_vx;
    vy = cmd_vy;
    vz = 0.0;

    

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "Errors: Pos(%.3f %.3f %.3f) Vel(%.3f %.3f %.3f) | "
                          "VelCmd(%.3f %.3f %.3f)",
                          ex, ey, ez, evx, evy, evz, vx, vy, vz);
  }

  void command_loop() {
    counter_++;

    // std::cout<<"global_traj_index_ "<<global_traj_index_<<endl;
    // std::cout<<"global_traj_size_ "<<global_traj_.size()<<endl;

    // 2. Iterate through the binary map
    // for (size_t idx = 0; idx < bin_map_.size(); ++idx) {

    //   // Check if the cell is an obstacle (assuming 1 represents an obstacle)
    //   if (bin_map_[idx] == 1) {

    //     // --- Requirement A: Index in Array ---
    //     // This is simply 'idx'

    //     // --- Requirement B: Grid Indexes (x, y) ---
    //     // Row-major order calculation
    //     int grid_x = idx % grid_w_;
    //     int grid_y = idx / grid_h_;

    //     // --- Requirement C: Location in World Frame (meters) ---
    //     // Formula: World = Origin + (Index * Resolution)
    //     // We add (resolution / 2.0) to get the center of the cell, rather than
    //     // the bottom-left corner. Vector2d world = grid_to_world(grid_x,
    //     // grid_y);

    //     // double world_x = world.x();
    //     // double world_y = world.y();

    //     // --- Print/Log the results ---
    //     // std::cout << "Obstacle Found:" << std::endl;
    //     // std::cout << "   1. Array Index: " << idx << std::endl;
    //     // std::cout << "   2. Grid Index : [X: " << grid_x << ", Y: " << grid_y
    //     // << "]" << std::endl; std::cout << "   3. World Pos  : (X: " <<
    //     // world_x << ", Y: " << world_y << ")" << std::endl; std::cout <<
    //     // "-----------------------------" << std::endl;
    //   }
    // }

    switch (stage_) {
    case FlightStage::WAIT_INIT:
      handle_wait_init();
      break;

    case FlightStage::TAKEOFF:
      handle_takeoff();
      break;

    case FlightStage::HOVER:
      handle_hover();
      break;

    case FlightStage::TRACKING:
      handle_tracking();
      break;
    }
  }

  void log_tracking_state() {
    // Log only if state changed
    if (tracking_state_ == last_tracking_state_)
      return;

    last_tracking_state_ = tracking_state_;

    switch (tracking_state_) {
    case TrackingState::NO_ODOM:
      RCLCPP_WARN(this->get_logger(),
                  "[TRACKING] NO_ODOM -> Switching to HOVER");
      break;

    case TrackingState::NO_TRAJECTORY:
      RCLCPP_WARN(this->get_logger(),
                  "[TRACKING] NO_TRAJECTORY -> Switching to HOVER");
      break;

    case TrackingState::TRACKING:
      RCLCPP_INFO(this->get_logger(), "[TRACKING] ACTIVE_TRACKING");
      break;

    case TrackingState::TRAJECTORY_FINISHED:
      RCLCPP_INFO(this->get_logger(),
                  "[TRACKING] TRAJECTORY_FINISHED -> Holding last point");
      break;

    case TrackingState::GOAL_REACHED:
      RCLCPP_INFO(this->get_logger(),
                  "[TRACKING] GOAL_REACHED -> Hovering at goal");
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "[TRACKING] UNKNOWN STATE");
      break;
    }
  }

  void handle_wait_init() {
    // Wait until MAVROS state + odom are valid, then mirror yaw_body_tracking
    // logic: SET_MODE -> ARM -> TAKEOFF (GUIDED takeoff to TAKEOFF_ALTITUDE),
    // then switch to TRACKING.
    if (!current_state_.connected || !has_odom_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "WAIT_INIT: Waiting for MAVROS (connected=%d has_odom=%d)",
          current_state_.connected, has_odom_);
      return;
    }

    // 1) Ensure GUIDED mode
    if (current_state_.mode != "GUIDED") {
      RCLCPP_INFO(get_logger(), "SET_MODE: requesting GUIDED");
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "GUIDED";
      mode_client_->async_send_request(req);
      return;
    }

    // 2) Ensure armed
    if (!current_state_.armed) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "ARM: sending arm command");
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      arm_client_->async_send_request(req);
      return;
    }

    // 3) Request takeoff once, then let handle_takeoff() watch altitude
    RCLCPP_INFO(get_logger(), "TAKEOFF: requesting altitude %.2f m",
                TAKEOFF_ALTITUDE);
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = TAKEOFF_ALTITUDE;
    takeoff_client_->async_send_request(req);

    stage_ = FlightStage::TAKEOFF;
  }

  void handle_takeoff() {
    double z = vehicle_odom_.position.z;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "TAKEOFF: z=%.2f (target=%.2f)", z, TAKEOFF_ALTITUDE);

    if (z >= TAKEOFF_ALTITUDE - 0.1) {
      hover_x_ = vehicle_odom_.position.x;
      hover_y_ = vehicle_odom_.position.y;
      hover_z_ = vehicle_odom_.position.z;

      RCLCPP_INFO(get_logger(),
                  "TAKEOFF: reached altitude -> TRACKING (z=%.2f)", z);
      stage_ = FlightStage::TRACKING;
    }
  }

  void handle_tracking() {

    if (replanning_) {
        publish_stop_velocity();
        return;
    }

    double vx, vy, vz;

    tracking_pp_controller(vx, vy, vz);

    // Use computed world-frame velocity command (ax, ay, az as v_x, v_y, v_z)
    publish_velocity_cmd(vx, vy, vz);

    global_traj_index_ += 1;
  }

  void handle_hover() {
    // Cannot hover without odometry
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Hover requested but no odometry available.");

      return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Entered HOVER mode at (%.2f, %.2f, %.2f)", hover_x_,
                         hover_y_, hover_z_);

    // In hover we simply command zero velocity; position is held by the
    // autopilot
    publish_stop_velocity();
  }
  void save_executed_path()
  {
      std::ofstream f("/workspace/src/px4_minco_trajectory_planner_ros2_package/scripts/executed_path.csv");

      f << "x,y,z\n";

      for (auto &p : executed_path_)
      {
          f << p.x() << "," << p.y() << "," << p.z() << "\n";
      }

      f.close();
  }
  void save_map()
  {
      std::ofstream f("/workspace/src/px4_minco_trajectory_planner_ros2_package/scripts/map.csv");

      for (int y = 0; y < grid_h_; y++)
      {
          for (int x = 0; x < grid_w_; x++)
          {
              int idx = y * grid_w_ + x;
              f << int(bin_map_[idx]) << ",";
          }
          f << "\n";
      }

      f.close();
  }
  void save_planned_traj()
  {
      std::ofstream f("/workspace/src/px4_minco_trajectory_planner_ros2_package/scripts/planned_traj.csv", std::ios::app);  // append mode

      if (!f.is_open())
          return;

      f << "\n";
   
      f << "==============================\n";
      f << "REPLAN_ID," << replan_id_ << "\n";
      f << "==============================\n";
      f << "t,x,y,z,vx,vy,vz,ax,ay,az\n";

      for (auto &p : global_traj_)
      {
          f << p.t << ","
            << p.p.x() << "," << p.p.y() << "," << p.p.z() << ","
            << p.v.x() << "," << p.v.y() << "," << p.v.z() << ","
            << p.a.x() << "," << p.a.y() << "," << p.a.z() << "\n";
      }

      f.close();
  }
};

/* ========================================================= */

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<IntegratedMappingSFCGCOPTER>());

  rclcpp::shutdown();

  return 0;
}