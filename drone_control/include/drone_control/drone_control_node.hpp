#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

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
  void takeoff(int height);
  void setup_subscribers();

  void local_pos_subscriber();
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  void callback_state(const mavros_msgs::msg::State::SharedPtr msg);

  void drone_state_subscriber();
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      local_pos_sub_;
  void callback_local_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_local_pos_;
};
