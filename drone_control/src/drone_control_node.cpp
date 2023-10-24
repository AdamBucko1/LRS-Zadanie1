#include <chrono>
#include <drone_control/drone_control_node.hpp>
#include <thread>

using namespace std::chrono_literals;

DroneControlNode::DroneControlNode() : Node("template_drone_control_node") {
  init();
}

void DroneControlNode::init() {
  start_takeoff_height_ = 0.25;
  spawn_position.x = 13.6;
  spawn_position.y = 1.5;
  spawn_position.z = 0;
  start_posisiton = spawn_position;
  start_posisiton.z = start_takeoff_height_;
  mission_state = INIT;
  path_waypoint_index_ = 0;
  waypoint_index_ = 0;
  // main_waypoints_.push_back(start_posisiton);
  std::vector<std::string> precision;
  std::vector<std::string> command;

  readCSVData("/home/adam/Downloads/points_example.csv", main_waypoints_,
              precision, command);
  std::cout << main_waypoints_[0 + 1].x << " - " << main_waypoints_[0 + 1].y
            << " - " << main_waypoints_[0 + 1].z << std::endl;
  std::cout << main_waypoints_[0].x << " - " << main_waypoints_[0].y << " - "
            << main_waypoints_[0].z << std::endl;
  RCLCPP_INFO(this->get_logger(), "Mission plan loaded");

  for (int waypoint_index_generate = 0;
       waypoint_index_generate < static_cast<int>(main_waypoints_.size());
       waypoint_index_generate++) {
    RCLCPP_INFO(this->get_logger(), "Path generation for waypoint %d ",
                waypoint_index_generate);

    if (waypoint_index_generate == 0) {
      map_handler_.generate_path(start_posisiton,
                                 main_waypoints_[waypoint_index_generate]);
    } else {
      map_handler_.generate_path(main_waypoints_[waypoint_index_generate - 1],
                                 main_waypoints_[waypoint_index_generate]);
    }

    for (Point<double> point : map_handler_.get_waypoints()) {
      path_waypoints_.push_back(std::vector<Point<double>>());
      path_waypoints_[waypoint_index_generate].push_back(point);
    }
    path_waypoints_[waypoint_index_generate].push_back(
        main_waypoints_[waypoint_index_generate]);
  }
  RCLCPP_INFO(this->get_logger(), "Path generated ");

  setup_publishers();
  setup_subscribers();
  setup_services();
  setup_clients();
}
void DroneControlNode::readCSVData(const std::string &filename,
                                   std::vector<Point<double>> &main_waypoints_,
                                   std::vector<std::string> &precision,
                                   std::vector<std::string> &command) {
  std::ifstream infile(filename);

  if (!infile.is_open()) {
    std::cerr << "Failed to open file " << filename << std::endl;
    return;
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    Point<double> pt;

    char delimiter;
    if (iss >> pt.x >> delimiter >> pt.y >> delimiter >> pt.z) {
      if (pt.z <= 0.25) {
        pt.z = 0.25;
      } else if (pt.z <= 0.75) {
        pt.z = 0.75;
      } else if (pt.z <= 0.80) {
        pt.z = 0.80;
      } else if (pt.z <= 1) {
        pt.z = 1;
      } else if (pt.z <= 1.25) {
        pt.z = 1.25;
      } else if (pt.z <= 1.5) {
        pt.z = 1.5;
      } else if (pt.z <= 1.75) {
        pt.z = 1.75;
      } else if (pt.z <= 1.8) {
        pt.z = 1.8;
      } else if (pt.z <= 2) {
        pt.z = 2;
      } else if (pt.z <= 2.25) {
        pt.z = 2.25;
      } else if (pt.z >= 2.25) {
        pt.z = 2.25;
      }
      main_waypoints_.push_back(pt);
    } else {
      continue;
    }

    std::string prec, cmd;
    if (iss >> delimiter >> prec >> delimiter >> cmd) {
      precision.push_back(prec);
      command.push_back(cmd);
    }
  }

  infile.close();
  return;
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

  // RCLCPP_INFO(this->get_logger(), "Autonomous mission called ");
  switch (mission_state) {
  case INIT:
    if (current_state_.connected) {
      mission_state = CONNECTED;
    }
    break;
  case CONNECTED:
    if (current_state_.mode == "GUIDED") {
      mission_state = GUIDED;
    } else {
      set_mode("GUIDED");
      RCLCPP_INFO(this->get_logger(), "Setting mode to GUIDED");
    }
    break;

  case GUIDED:
    if (current_state_.armed) {
      mission_state = ARMED;
    } else {
      arm_throttle();
      RCLCPP_INFO(this->get_logger(), "Arming motors");
    }
    break;

  case ARMED:
    RCLCPP_INFO(this->get_logger(), "sending takeoff");
    takeoff(start_takeoff_height_);
    if (abs(current_local_pos_.pose.position.z - start_takeoff_height_) <
        0.08) {
      mission_state = FLYING;
      RCLCPP_INFO(this->get_logger(), "drone took off");
      select_next_waypoint(path_waypoint_, path_waypoints_[waypoint_index_],
                           path_waypoint_index_, false);
      int throw_away_index = -1;
      select_next_waypoint(waypoint_location_, main_waypoints_,
                           throw_away_index, false);
    }

    break;

  case FLYING:
    if (check_waypoint_reached(path_waypoint_, 0.05)) {
      if (!select_next_waypoint(path_waypoint_,
                                path_waypoints_[waypoint_index_],
                                path_waypoint_index_, false)) {
        RCLCPP_INFO(this->get_logger(), "Reached end of sequential path");
      }
    }
    if (check_waypoint_reached(waypoint_location_, 0.08)) {
      RCLCPP_INFO(this->get_logger(), "Waypoint reached");
      mission_state = WAYPOINT_REACHED;
    }
    waypoint_pose_pub_->publish(path_waypoint_);
    break;

  case WAYPOINT_REACHED:
    perform_waypoint_action();

  case ACTION_PERFORMED:
    if (select_next_waypoint(waypoint_location_, main_waypoints_,
                             waypoint_index_, true)) {
      mission_state = FLYING;
      path_waypoint_index_ = 0;
    } else {
      mission_state = FINISHED;
      RCLCPP_INFO(this->get_logger(), "FINISHED");
    }
    break;

  default:
    RCLCPP_INFO(this->get_logger(), "Nothing performed");
    break;
  }
}

bool DroneControlNode::select_next_waypoint(
    geometry_msgs::msg::PoseStamped &waypoint_location_,
    std::vector<Point<double>> waypoints, int &index, bool main_waypoint) {
  if (index == waypoints.size() - 1) {
    if (main_waypoint)
      RCLCPP_INFO(this->get_logger(), "Last main waypoint reached");
    if (!main_waypoint)
      RCLCPP_INFO(this->get_logger(), "Completed path between two waypoints");
    return false;

  } else {
    index++;
    waypoint_location_.pose.position.x = waypoints[index].x;
    waypoint_location_.pose.position.y = waypoints[index].y;
    waypoint_location_.pose.position.z = waypoints[index].z;
    if (main_waypoint) {
      RCLCPP_INFO(this->get_logger(),
                  "Selected next main waypoint number: %d with coordinates x: "
                  "%f, y: %f, z: %f",
                  index, waypoint_location_.pose.position.x,
                  waypoint_location_.pose.position.y,
                  waypoint_location_.pose.position.z);
    }
    if (!main_waypoint) {
      RCLCPP_INFO(this->get_logger(),
                  "Selected next partial waypoint number: %d with coordinates "
                  "x: %f, y: %f, z: %f",
                  index, waypoint_location_.pose.position.x,
                  waypoint_location_.pose.position.y,
                  waypoint_location_.pose.position.z);
    }
    return true;
  }
}

bool DroneControlNode::check_waypoint_reached(
    geometry_msgs::msg::PoseStamped waypoint, double tolerance) {
  double dx = current_local_pos_.pose.position.x - waypoint.pose.position.x;
  double dy = current_local_pos_.pose.position.y - waypoint.pose.position.y;
  double dz = current_local_pos_.pose.position.z - waypoint.pose.position.z;

  if (std::abs(dx) <= tolerance && std::abs(dy) <= tolerance &&
      std::abs(dz) <= tolerance) {
    return true;
  }
  return false;
}
void DroneControlNode::perform_waypoint_action() {
  RCLCPP_INFO(this->get_logger(), "Action performed");
  mission_state = ACTION_PERFORMED;
}
void DroneControlNode::set_mode(std::string mode) {

  mavros_msgs::srv::SetMode::Request guided_set_mode_req;
  guided_set_mode_req.custom_mode = mode;

  while (!set_mode_client_->wait_for_service(1s)) {
    // RCLCPP_INFO(this->get_logger(), "test 1");
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
    // RCLCPP_INFO(this->get_logger(), "test 2");
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
    // RCLCPP_INFO(this->get_logger(), "test 3");
    if (!rclcpp::ok()) {
      //   RCLCPP_INFO(this->get_logger(), "test 5");
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the takeoff service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
  }
  // RCLCPP_INFO(this->get_logger(), "test 6");
  // send a request to take off
  auto result_future = takeoff_client_->async_send_request(
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_req));
  // RCLCPP_INFO(this->get_logger(), "test 7");
  // Wait for the future with a timeout
  if (result_future.wait_for(1s) == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff response.");
    return; // Exit the function
  }

  // RCLCPP_INFO(this->get_logger(), "test 8");

  // Check the result
  auto result = result_future.get();
  // RCLCPP_INFO(this->get_logger(), "test 9");
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to take off");
    //   RCLCPP_INFO(this->get_logger(), "test 10");
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
  // RCLCPP_INFO(this->get_logger(), "test 4");
  current_local_pos_ = *msg;
  autonomous_mission();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControlNode>());
  rclcpp::shutdown();
  return 0;
}
