#include "drone_control/map_handler.hpp"
#include <drone_control/map_handler.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_srvs/srv/set_bool.hpp> // For the SetBool service type
class DroneControlNode : public rclcpp::Node {
public:
  DroneControlNode();

private:
  void init();
  void setup_publishers();
  void setup_subscribers();
  void setup_services();
  void setup_clients();
  void connect_to_mavros();
  void set_mode(std::string mode);
  void arm_throttle();
  void takeoff(double height);
  void initialize_trajectory();
  void generate_path_waypoints();
  void load_data();
  void set_default_variables();
  void setup_service_servers();
  bool select_next_waypoint(geometry_msgs::msg::PoseStamped &waypoint_location_,
                            std::vector<Point<double>> waypoints, int &index,
                            bool main_waypoint);
  void perform_waypoint_action();
  void readCSVData(const std::string &filename,
                   std::vector<Point<double>> &main_waypoints,
                   std::vector<std::string> &precision,
                   std::vector<std::string> &command);

  void stop_service_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void continue_service_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_service;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr continue_service;

  void local_pos_subscriber();
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  void callback_state(const mavros_msgs::msg::State::SharedPtr msg);
  std::vector<double>
  getRPYFromQuaternion(const geometry_msgs::msg::Quaternion &q);
  void drone_state_subscriber();
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      local_pos_sub_;
  void callback_local_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void autonomous_mission();
  bool check_waypoint_reached(geometry_msgs::msg::PoseStamped waypoint,
                              double tolerance);
  void transformPoints(std::vector<Point<double>> &waypoints, double offset_x,
                       double offset_y);
  void set_precision(std::string &input);
  void select_waypoint_action();
  geometry_msgs::msg::Quaternion
  getQuaternionFromRPY(const std::vector<double> &rpy);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      waypoint_pose_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_local_pos_;
  geometry_msgs::msg::PoseStamped waypoint_location_;
  geometry_msgs::msg::PoseStamped path_waypoint_;
  bool taken_off_ = false;
  int waypoint_index_;
  int path_waypoint_index_;
  std::vector<std::vector<Point<double>>> path_waypoints_;
  std::vector<Point<double>> main_waypoints_;
  Point<double> start_posisiton;
  Point<double> spawn_position;
  double start_takeoff_height_;
  double precision_;
  double mission_yaw_;
  std::vector<std::string> precision_vect_;
  std::vector<std::string> command_vect_;
  std::vector<double> current_rpy;
  enum MissionState {
    INIT,
    CONNECTED,
    GUIDED,
    ARMED,
    FLYING,
    WAYPOINT_REACHED,
    ACTION_PERFORMED,
    FINISHED,
    STOP
  };

  enum ActionState { NONE, LANDTAKEOFF, YAW, LAND, LANDED };
  MissionState mission_state;
  ActionState action_state;
  MissionState remembered_mission_state;
  ActionState remembered_action_state;
  MapHandler map_handler_;
};
