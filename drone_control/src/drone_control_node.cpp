#include "drone_control/drone_control_node.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

DroneControlNode::DroneControlNode() : Node("template_drone_control_node") {
  init();
}

void DroneControlNode::init() {
  waypoint_location_.pose.position.x = 1;
  waypoint_location_.pose.position.y = 1;
  waypoint_location_.pose.position.z = 1;
  waypoint_index_ = 0;
  std::vector<Point<double>> waypoints = function();
  setup_publishers();
  setup_subscribers();
  setup_services();
  setup_clients();
}

void DroneControlNode::setup_publishers() {
  waypoint_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);
}

void DroneControlNode::setup_subscribers() {
  drone_state_subscriber();
  local_pos_subscriber();
}
void DroneControlNode::drone_state_subscriber() {
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&DroneControlNode::callback_state, this,
                std::placeholders::_1));
}
void DroneControlNode::local_pos_subscriber() {
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = 1;
  custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos =
      rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
  local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos,
      std::bind(&DroneControlNode::callback_local_pos, this,
                std::placeholders::_1));
}

void DroneControlNode::setup_clients() {
  arming_client_ =
      this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  set_mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  takeoff_client_ =
      this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
}

void DroneControlNode::setup_services() {}

void DroneControlNode::autonomous_mission() {

  RCLCPP_INFO(this->get_logger(), "Autonomous mission called ");
  if (current_state_.mode != "GUIDED") {
    set_mode("GUIDED");
    RCLCPP_INFO(this->get_logger(), "Setting mode to GUIDED");
    return;
  }
  if (!current_state_.armed) {
    arm_throttle();
    RCLCPP_INFO(this->get_logger(), "Arming motors");
    return;
  }
  if (!taken_off_) {
    RCLCPP_INFO(this->get_logger(), "sending takeoff");
    takeoff(0.25);
    if (!taken_off_) {
      std::this_thread::sleep_for(1s);
      taken_off_ = true;
    }
  }
  if (taken_off_) {
    if (go_to_waypoint(waypoint_location_, 0.05)) {
      perform_waypoint_action();
      if (waypoint_index_ == waypoints.size()) {
        RCLCPP_INFO(this->get_logger(), "Last Waypoint Reached");
        return;
      }
      waypoint_index_++;
      waypoint_location_.pose.position.x = waypoints[waypoint_index_].x;
      waypoint_location_.pose.position.y = waypoints[waypoint_index_].y;
      waypoint_location_.pose.position.z = waypoints[waypoint_index_].z;
    }
    waypoint_pose_pub_->publish(waypoint_location_);
  }
  RCLCPP_INFO(this->get_logger(), "nothing sent");
}
bool DroneControlNode::go_to_waypoint(geometry_msgs::msg::PoseStamped waypoint,
                                      double tolerance) {
  double dx = current_local_pos_.pose.position.x - waypoint.pose.position.x;
  double dy = current_local_pos_.pose.position.y - waypoint.pose.position.y;
  double dz = current_local_pos_.pose.position.z - waypoint.pose.position.z;

  if (std::abs(dx) <= tolerance && std::abs(dy) <= tolerance &&
      std::abs(dz) <= tolerance) {
    return true;
  }
  return false;
}
void DroneControlNode::perform_waypoint_action() {}
void DroneControlNode::set_mode(std::string mode) {

  mavros_msgs::srv::SetMode::Request guided_set_mode_req;
  guided_set_mode_req.custom_mode = mode;

  while (!set_mode_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "test 1");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the set_mode service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
  }
  auto result = set_mode_client_->async_send_request(
      std::make_shared<mavros_msgs::srv::SetMode::Request>(
          guided_set_mode_req));
}

void DroneControlNode::arm_throttle() {

  mavros_msgs::srv::CommandBool::Request arming_req;
  arming_req.value = true; // true to arm the drone

  while (!arming_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "test 2");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the arming service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
  }
  auto result_future = arming_client_->async_send_request(
      std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming_req));
}

void DroneControlNode::takeoff(double height) {

  mavros_msgs::srv::CommandTOL::Request takeoff_req;
  takeoff_req.altitude =
      height; // set the height you want the drone to take off to

  // wait for the takeoff service to become available
  while (!takeoff_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "test 3");
    if (!rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "test 5");
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the takeoff service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
  }
  RCLCPP_INFO(this->get_logger(), "test 6");
  // send a request to take off
  auto result_future = takeoff_client_->async_send_request(
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_req));
  RCLCPP_INFO(this->get_logger(), "test 7");
  // Wait for the future with a timeout
  if (result_future.wait_for(1s) == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff response.");
    return; // Exit the function
  }

  RCLCPP_INFO(this->get_logger(), "test 8");

  // Check the result
  auto result = result_future.get();
  RCLCPP_INFO(this->get_logger(), "test 9");
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to take off");
    RCLCPP_INFO(this->get_logger(), "test 10");
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully initiated takeoff");
    taken_off_ = true;
  }
}

void DroneControlNode::callback_state(
    const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Current State: %s",
              current_state_.mode.c_str());
}

void DroneControlNode::callback_local_pos(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "test 4");
  current_local_pos_ = *msg;
  autonomous_mission();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControlNode>());
  rclcpp::shutdown();
  return 0;
}
