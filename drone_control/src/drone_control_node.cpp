#include "drone_control/drone_control_node.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

DroneControlNode::DroneControlNode() : Node("template_drone_control_node") {
  init();
}

void DroneControlNode::init() {
  setup_publishers();
  setup_subscribers();
  setup_services();
  setup_clients();
  connect_to_mavros();
}

void DroneControlNode::setup_publishers() {
  local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);
}

void DroneControlNode::setup_subscribers() {
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&DroneControlNode::callback_state, this,
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

void DroneControlNode::connect_to_mavros() {

  while (rclcpp::ok() && !current_state_.connected) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
  }
}

void DroneControlNode::set_mode(std::string mode) {

  mavros_msgs::srv::SetMode::Request guided_set_mode_req;
  guided_set_mode_req.custom_mode = mode;

  while (!set_mode_client_->wait_for_service(1s)) {
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

void DroneControlNode::takeoff(int height) {

  mavros_msgs::srv::CommandTOL::Request takeoff_req;
  takeoff_req.altitude =
      height; // set the height you want the drone to take off to

  // wait for the takeoff service to become available
  while (!takeoff_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the takeoff service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
  }

  // send a request to take off
  auto result_future = takeoff_client_->async_send_request(
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_req));

  // handle the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    // Assuming there is a 'success' field in result to check if the command was
    // successful
    if (!result->success)
      RCLCPP_ERROR(this->get_logger(), "Failed to take off");
    else
      RCLCPP_INFO(this->get_logger(), "Successfully initiated takeoff");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call takeoff service");
  }
}

void DroneControlNode::callback_state(
    const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Current State: %s",
              current_state_.mode.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControlNode>());
  rclcpp::shutdown();
  return 0;
}
