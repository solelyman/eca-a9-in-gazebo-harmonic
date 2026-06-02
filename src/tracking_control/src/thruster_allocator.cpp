#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm>
#include <array>

class ThrusterAllocator : public rclcpp::Node
{
public:
  ThrusterAllocator() : Node("thruster_allocator")
  {
    this->declare_parameter("max_fin_angle", 0.523);
    this->declare_parameter("fin_lift_coefficient", 3.0);
    this->declare_parameter("thruster_rotor_coefficient", 0.002);

    max_fin_angle_ = this->get_parameter("max_fin_angle").as_double();
    c_lift_ = this->get_parameter("fin_lift_coefficient").as_double();
    c_rotor_ = this->get_parameter("thruster_rotor_coefficient").as_double();

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_callback(msg); });

    pub_thruster_ = this->create_publisher<std_msgs::msg::Float64>(
      "joint/thruster_joint/cmd_thrust", 10);

    for (int i = 0; i < 4; ++i) {
      pub_fins_[i] = this->create_publisher<std_msgs::msg::Float64>(
        "joint/fin" + std::to_string(i) + "_joint/cmd_pos", 10);
    }

    RCLCPP_INFO(this->get_logger(), "Thruster Allocator Initialized.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double thrust_force = msg->linear.x;
    double pitch_torque = msg->angular.y;
    double yaw_torque   = msg->angular.z;

    if (thrust_force < 0.0) thrust_force = 0.0;
    double thrust_cmd = 0.0;
    if (std::abs(thrust_force) > 1e-6) {
      double sign = (thrust_force > 0) ? 1.0 : -1.0;
      thrust_cmd = sign * std::sqrt(std::abs(thrust_force) / c_rotor_);
    }
    thrust_cmd = std::clamp(thrust_cmd, -1500.0, 1500.0);

    double delta_pitch = pitch_torque / c_lift_;
    double delta_yaw   = yaw_torque   / c_lift_;

    double fin0 =  delta_pitch + delta_yaw;
    double fin1 =  delta_pitch - delta_yaw;
    double fin2 = -delta_pitch - delta_yaw;
    double fin3 = -delta_pitch + delta_yaw;

    fin0 = std::clamp(fin0, -max_fin_angle_, max_fin_angle_);
    fin1 = std::clamp(fin1, -max_fin_angle_, max_fin_angle_);
    fin2 = std::clamp(fin2, -max_fin_angle_, max_fin_angle_);
    fin3 = std::clamp(fin3, -max_fin_angle_, max_fin_angle_);

    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = thrust_cmd;
    pub_thruster_->publish(thrust_msg);

    double fins[4] = {fin0, fin1, fin2, fin3};
    for (int i = 0; i < 4; ++i) {
      std_msgs::msg::Float64 fin_msg;
      fin_msg.data = fins[i];
      pub_fins_[i]->publish(fin_msg);
    }
  }

  double max_fin_angle_;
  double c_lift_;
  double c_rotor_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_thruster_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> pub_fins_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterAllocator>());
  rclcpp::shutdown();
  return 0;
}
