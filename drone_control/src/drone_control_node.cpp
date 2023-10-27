#include <chrono>
#include <drone_control/drone_control_node.hpp>
#include <thread>

using namespace std::chrono_literals;

DroneControlNode::DroneControlNode() : Node("template_drone_control_node") {
  init();
}

void DroneControlNode::init() {
  set_default_variables();
  load_data();
  generate_path_waypoints();
  initialize_trajectory();
  setup_publishers();
  setup_subscribers();
  setup_services();
  setup_clients();
}

void DroneControlNode::initialize_trajectory() {
  std::cout << precision_vect_[0] << std::endl;
  set_precision(precision_vect_[0]);
  path_waypoint_index_ = -1;
  select_next_waypoint(path_waypoint_, path_waypoints_[waypoint_index_],
                       path_waypoint_index_, false);
  waypoint_index_ = -1;
  select_next_waypoint(waypoint_location_, main_waypoints_, waypoint_index_,
                       false);
}

void DroneControlNode::generate_path_waypoints() {
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
    transformPoints(path_waypoints_[waypoint_index_generate], spawn_position.x,
                    spawn_position.y);
  }
  transformPoints(main_waypoints_, spawn_position.x, spawn_position.y);

  RCLCPP_INFO(this->get_logger(), "Path generated ");
}

void DroneControlNode::load_data() {
  readCSVData("/home/adam/Downloads/points_example.csv", main_waypoints_,
              precision_vect_, command_vect_);
  std::cout << main_waypoints_[0 + 1].x << " - " << main_waypoints_[0 + 1].y
            << " - " << main_waypoints_[0 + 1].z << std::endl;
  std::cout << main_waypoints_[0].x << " - " << main_waypoints_[0].y << " - "
            << main_waypoints_[0].z << std::endl;
  RCLCPP_INFO(this->get_logger(), "Mission plan loaded");
}

void DroneControlNode::set_default_variables() {
  start_takeoff_height_ = 0.25;
  spawn_position.x = 13.6;
  spawn_position.y = 1.5;
  spawn_position.z = 0;
  start_posisiton = spawn_position;
  start_posisiton.z = start_takeoff_height_;
  mission_state = INIT;
  action_state = NONE;
  path_waypoint_index_ = 0;
  waypoint_index_ = 0;
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
    std::string value;

    // Parsing x
    if (!std::getline(iss, value, ','))
      break;
    pt.x = std::stod(value);

    // Parsing y
    if (!std::getline(iss, value, ','))
      break;
    pt.y = std::stod(value);

    // Parsing z
    if (!std::getline(iss, value, ','))
      break;
    pt.z = std::stod(value);

    // Adjusting pt.z value
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

    // Parsing precision
    if (!std::getline(iss, value, ','))
      break;
    precision.push_back(value);
    std::cout << "Parsed precision: " << value << std::endl;

    // Parsing command
    if (!std::getline(iss, value))
      break;
    command.push_back(value);
    std::cout << "Parsed command: " << value << std::endl;
  }

  infile.close();
}

void DroneControlNode::transformPoints(std::vector<Point<double>> &waypoints,
                                       double offset_x, double offset_y) {
  for (auto &point : waypoints) {
    double new_x = -point.y + offset_y;
    double new_y = point.x - offset_x;

    point.x = new_x;
    point.y = new_y;
  }
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
    RCLCPP_INFO(this->get_logger(), "Sending takeoff to %f",
                start_takeoff_height_);
    takeoff(start_takeoff_height_);
    if (abs(current_local_pos_.pose.position.z - start_takeoff_height_) < 0.1) {
      mission_state = FLYING;
      RCLCPP_INFO(this->get_logger(), "Drone took off");
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
    if (check_waypoint_reached(waypoint_location_, precision_)) {
      RCLCPP_INFO(this->get_logger(), "Waypoint reached");
      mission_state = WAYPOINT_REACHED;
    }
    waypoint_pose_pub_->publish(path_waypoint_);
    break;

  case WAYPOINT_REACHED:
    select_waypoint_action();
    perform_waypoint_action();
    break;

  case ACTION_PERFORMED:
    RCLCPP_INFO(this->get_logger(), "ACTION PERFORMED STATE");
    if (select_next_waypoint(waypoint_location_, main_waypoints_,
                             waypoint_index_, true)) {
      mission_state = FLYING;
      path_waypoint_index_ = 0;
      set_precision(precision_vect_[waypoint_index_]);
    } else {
      mission_state = FINISHED;
      RCLCPP_INFO(this->get_logger(), "FINISHED");
    }
    if (action_state == LANDTAKEOFF) {
      action_state = NONE;
      mission_state = CONNECTED;
    }
    break;

  default:
    RCLCPP_INFO(this->get_logger(), "Nothing performed");
    break;
  }
}

void DroneControlNode::set_precision(std::string &input) {
  std::string lowercaseInput = input;

  std::transform(input.begin(), input.end(), lowercaseInput.begin(), ::tolower);
  if (lowercaseInput.find("hard") != std::string::npos) {
    precision_ = 0.05;
    RCLCPP_INFO(this->get_logger(),
                "The next waypoint will be reached with hard precision");
  } else if (lowercaseInput.find("soft") != std::string::npos) {
    precision_ = 0.10;
    RCLCPP_INFO(this->get_logger(),
                "The next waypoint will be reached with soft precision");

  } else {
    std::cout << "The string doesn't mention 'soft' or 'hard'." << std::endl;
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
    // if (main_waypoint) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Selected next main waypoint number: %d with coordinates x:
    //               "
    //               "%f, y: %f, z: %f",
    //               index, waypoint_location_.pose.position.x,
    //               waypoint_location_.pose.position.y,
    //               waypoint_location_.pose.position.z);
    // }
    // if (!main_waypoint) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Selected next partial waypoint number: %d with coordinates
    //               " "x: %f, y: %f, z: %f", index,
    //               waypoint_location_.pose.position.x,
    //               waypoint_location_.pose.position.y,
    //               waypoint_location_.pose.position.z);
    // }
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
void DroneControlNode::select_waypoint_action() {
  if (command_vect_[waypoint_index_] == "land") {
    action_state = LAND;
    RCLCPP_INFO(this->get_logger(), "LAND action selected");
  } else if (command_vect_[waypoint_index_] == "landtakeoff") {
    action_state = LANDTAKEOFF;
    RCLCPP_INFO(this->get_logger(), "LANDTAKEOFF action selected");
  } else if (command_vect_[waypoint_index_] == "-") {
    action_state = NONE;
    RCLCPP_INFO(this->get_logger(), "No action on waypoint");
  } else if (std::regex_match(command_vect_[waypoint_index_],
                              std::regex("yaw\\d+"))) {
    // Handle the yaw[number] command
    std::smatch match;
    std::regex_search(command_vect_[waypoint_index_], match,
                      std::regex("\\d+"));
    mission_yaw_ = std::stoi(match[0].str());
    action_state = YAW;
  } else {
    std::cerr << "Unknown command: " << command_vect_[waypoint_index_]
              << std::endl;
    action_state = NONE;
  }
}
void DroneControlNode::perform_waypoint_action() {

  switch (action_state) {
  case LAND:
    if (current_state_.mode != "LAND") {
      set_mode("LAND");
      RCLCPP_INFO(this->get_logger(), "Setting land mode");
    }
    if (!current_state_.armed) {
      mission_state = FINISHED;
      action_state = NONE;
    }
    break;
  case LANDTAKEOFF:
    if (current_state_.mode != "LAND") {
      set_mode("LAND");
      RCLCPP_INFO(this->get_logger(), "Setting land mode");
      start_takeoff_height_ = main_waypoints_[waypoint_index_].z;
    }
    if (!current_state_.armed) {
      RCLCPP_INFO(this->get_logger(),
                  "Drone is unarmed therefore on the ground");
      mission_state = ACTION_PERFORMED;
    }
    break;
  case YAW:
    RCLCPP_INFO(this->get_logger(), "Set yaw to %f", mission_yaw_);
    current_rpy = getRPYFromQuaternion(current_local_pos_.pose.orientation);
    if (abs(current_rpy[2] - mission_yaw_ * M_PI / 180.0) < 5 * M_PI / 180.0) {
      mission_state = ACTION_PERFORMED;
      action_state = NONE;
    } else {
      RCLCPP_INFO(this->get_logger(), "Mission yaw %f, current yaw %f",
                  mission_yaw_, current_rpy[2]);
      std::vector<double> wanted_rpy = {0, 0, 90 * M_PI / 180.0};
      path_waypoint_.pose.orientation = getQuaternionFromRPY(wanted_rpy);
      waypoint_pose_pub_->publish(path_waypoint_);
    }
    break;

  default:
    break;
  }

  if (action_state == NONE) {
    RCLCPP_INFO(this->get_logger(), "Set here ");
    mission_state = ACTION_PERFORMED;
  }
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
    // RCLCPP_ERROR(this->get_logger(), "Timeout waiting for takeoff
    // response.");
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
  // RCLCPP_INFO(this->get_logger(), "Current State: %s",
  //             current_state_.mode.c_str());
}

void DroneControlNode::callback_local_pos(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "test 4");
  current_local_pos_ = *msg;
  autonomous_mission();
}

std::vector<double> DroneControlNode::getRPYFromQuaternion(
    const geometry_msgs::msg::Quaternion &q) {
  std::vector<double> rpy(3); // Initialize a vector with 3 elements

  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::fabs(sinp) >= 1)
    rpy[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    rpy[1] = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);

  return rpy;
}

geometry_msgs::msg::Quaternion
DroneControlNode::getQuaternionFromRPY(const std::vector<double> &rpy) {
  geometry_msgs::msg::Quaternion q;

  double roll = rpy[0];
  double pitch = rpy[1];
  double yaw = rpy[2];

  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControlNode>());
  rclcpp::shutdown();
  return 0;
}
