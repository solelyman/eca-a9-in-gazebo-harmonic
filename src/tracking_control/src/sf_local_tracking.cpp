#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class CommandFilter{
private:
    double omegan_, zetan_, limit_, x1_, x2_;
public:
    CommandFilter() : omegan_(1.0), zetan_(0.7), limit_(1.0), x1_(0.0), x2_(0.0) {}
    CommandFilter(double omegan, double zetan, double limit): 
        omegan_(omegan), zetan_(zetan), limit_(limit), x1_(0.0), x2_(0.0){}

    void reset(double val){
        x1_ = std::clamp(val, -limit_, limit_);
        x2_ = 0.0;
    }
    
    void update(double input, double dt){
        double input_bounded = std::clamp(input, -limit_, limit_);
        double error = x1_ - input_bounded;
        double x2_dot = -2.0 * zetan_ * omegan_ * x2_ - omegan_ * omegan_ * error;
        x2_ += x2_dot * dt;
        x1_ += x2_ * dt;
    }
    
    double getSpeed() const {return x1_;}
    double getRate() const {return x2_;}
};

class SFController : public rclcpp::Node
{
private:
    CommandFilter yaw_filter_;
    CommandFilter theta_filter_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ref_, sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_final_goal_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_pred_current_; 
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry current_ref_;
    geometry_msgs::msg::Point final_goal_;
    rclcpp::Time last_time_; 
    
    double x_, y_, z_, phi_, theta_, psi_;
    double u_, v_, w_, p_, q_, r_;
    bool has_final_goal_ = false;
    
    // Prediction
    double pred_u_ = 0.0;
    double pred_v_ = 0.0;
    double pred_w_ = 0.0;
    bool has_prediction_ = false;

    // Parameters
    double last_yaw_ref_ = 0.0; 
    double last_pitch_ref_ = 0.0;
    double omegan_, zetan_;
    double delta_h_, delta_v_, k_x_, k_u_, k_theta_, k_q_, k_psi_, k_r_;
    double stop_radius_;
    
    double m11_, m22_, m33_, I_yy_, I_zz_;
    double d_u_, d_q_, d_r_;
    double mass_, volume_, rho_, cob_z_, gravity_;
    double max_tau_u_, max_tau_q_, max_tau_r_;
    double max_u_, max_q_, max_r_;
    
    bool has_ref_ = false;
    bool has_odom_ = false;
    bool first_run_ = true;

    void declare_params(){
        this->declare_parameter("vehicle_model.mass_11", 4.0);
        this->declare_parameter("vehicle_model.mass_22", 95.0);
        this->declare_parameter("vehicle_model.mass_33", 75.0);
        this->declare_parameter("vehicle_model.inertia_yy", 30.0);
        this->declare_parameter("vehicle_model.inertia_zz", 35.0);
        this->declare_parameter("vehicle_model.damping_11", -8.0);
        this->declare_parameter("vehicle_model.damping_55", -20.0);
        this->declare_parameter("vehicle_model.damping_66", -20.0);
        this->declare_parameter("vehicle_model.mass", 100.0);
        this->declare_parameter("vehicle_model.volume", 0.098);
        this->declare_parameter("vehicle_model.rho", 1025.0);
        this->declare_parameter("vehicle_model.cob_z", 0.06);
        this->declare_parameter("vehicle_model.gravity", 9.81);
        
        this->declare_parameter("dynamics_limits.max_thrust", 300.0);
        this->declare_parameter("dynamics_limits.max_torque", 100.0);
        this->declare_parameter("dynamics_limits.max_surge_speed", 2.0);
        this->declare_parameter("dynamics_limits.max_yaw_rate", 1.0);
        this->declare_parameter("dynamics_limits.max_pitch_rate", 1.0);
        
        this->declare_parameter("controller_gains.lookahead_h", 12.0);
        this->declare_parameter("controller_gains.lookahead_v", 6.0);
        this->declare_parameter("controller_gains.k_x", 0.5);
        this->declare_parameter("controller_gains.k_u", 2.0);
        this->declare_parameter("controller_gains.k_theta", 4.0);
        this->declare_parameter("controller_gains.k_q", 10.0);
        this->declare_parameter("controller_gains.k_psi", 0.5);
        this->declare_parameter("controller_gains.k_r", 1.0);
        this->declare_parameter("controller_gains.stop_radius", 1.0);
        
        this->declare_parameter("filter.omega_n", 1.0);
        this->declare_parameter("filter.zeta_n", 0.707);
    }

    void load_params_and_init(){
        max_tau_u_ = this->get_parameter("dynamics_limits.max_thrust").as_double();
        max_tau_q_ = this->get_parameter("dynamics_limits.max_torque").as_double();
        max_tau_r_ = max_tau_q_;
        max_u_ = this->get_parameter("dynamics_limits.max_surge_speed").as_double();
        max_q_ = this->get_parameter("dynamics_limits.max_pitch_rate").as_double();
        max_r_ = this->get_parameter("dynamics_limits.max_yaw_rate").as_double();
        
        omegan_ = this->get_parameter("filter.omega_n").as_double();
        zetan_ = this->get_parameter("filter.zeta_n").as_double();
        
        yaw_filter_ = CommandFilter(omegan_, zetan_, max_r_);
        theta_filter_ = CommandFilter(omegan_, zetan_, max_q_);
        
        delta_h_ = this->get_parameter("controller_gains.lookahead_h").as_double();
        delta_v_ = this->get_parameter("controller_gains.lookahead_v").as_double();
        
        m11_ = this->get_parameter("vehicle_model.mass_11").as_double();
        m22_ = this->get_parameter("vehicle_model.mass_22").as_double();
        m33_ = this->get_parameter("vehicle_model.mass_33").as_double();
        I_yy_ = this->get_parameter("vehicle_model.inertia_yy").as_double();
        I_zz_ = this->get_parameter("vehicle_model.inertia_zz").as_double();
        
        d_u_ = this->get_parameter("vehicle_model.damping_11").as_double(); 
        d_q_ = this->get_parameter("vehicle_model.damping_55").as_double();
        d_r_ = this->get_parameter("vehicle_model.damping_66").as_double();
        
        mass_ = this->get_parameter("vehicle_model.mass").as_double();
        volume_ = this->get_parameter("vehicle_model.volume").as_double();
        rho_ = this->get_parameter("vehicle_model.rho").as_double();
        cob_z_ = this->get_parameter("vehicle_model.cob_z").as_double();
        gravity_ = this->get_parameter("vehicle_model.gravity").as_double();
        
        k_x_ = this->get_parameter("controller_gains.k_x").as_double();
        k_u_ = this->get_parameter("controller_gains.k_u").as_double();
        k_theta_ = this->get_parameter("controller_gains.k_theta").as_double();
        k_q_ = this->get_parameter("controller_gains.k_q").as_double();
        k_psi_ = this->get_parameter("controller_gains.k_psi").as_double();
        k_r_ = this->get_parameter("controller_gains.k_r").as_double();
        stop_radius_ = this->get_parameter("controller_gains.stop_radius").as_double();
    }
    // reference judgement
    void ref_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        current_ref_ = *msg;
        has_ref_ = true;
    }
    //recent state
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(phi_, theta_, psi_);
        
        u_ = msg->twist.twist.linear.x;
        v_ = msg->twist.twist.linear.y;
        w_ = msg->twist.twist.linear.z;
        p_ = msg->twist.twist.angular.x;
        q_ = msg->twist.twist.angular.y;
        r_ = msg->twist.twist.angular.z;
        has_odom_ = true;
    }
    //current prediction
    void prediction_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        pred_u_ = msg->linear.x;
        pred_v_ = msg->linear.y;
        pred_w_ = msg->linear.z;
        has_prediction_ = true;
    }

    double normalize_angle(double angle){
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void control_loop(){
        if(!has_odom_ || !has_ref_) return;
        //Initialization
        rclcpp::Time current_time = this->get_clock()->now();
        if (first_run_) {
            last_time_ = current_time;
            first_run_ = false;
            yaw_filter_.reset(r_);
            theta_filter_.reset(q_);
            return;
        }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        dt = std::clamp(dt, 0.001, 0.1);

        double x_ref = current_ref_.pose.pose.position.x;
        double y_ref = current_ref_.pose.pose.position.y;
        double z_ref = current_ref_.pose.pose.position.z;
        double u_ref = current_ref_.twist.twist.linear.x;

        tf2::Quaternion q_ref_tf(
            current_ref_.pose.pose.orientation.x, current_ref_.pose.pose.orientation.y,
            current_ref_.pose.pose.orientation.z, current_ref_.pose.pose.orientation.w);
        double roll_ref, pitch_ref, yaw_ref;
        tf2::Matrix3x3(q_ref_tf).getRPY(roll_ref, pitch_ref, yaw_ref);

        double dx = x_ - x_ref;
        double dy = y_ - y_ref;
        double dz = z_ - z_ref;

        double cp = cos(pitch_ref); double sp = sin(pitch_ref);
        double cy = cos(yaw_ref); double sy = sin(yaw_ref);
        
        double xe = cy * cp * dx + sy * cp * dy - sp * dz;
        double ye = -sy * dx + cy * dy;
        double ze = cy * sp * dx + sy * sp * dy + cp * dz;

        double theta_los = atan2(ze, delta_v_);
        if (z_ > -0.5) theta_los = std::max(theta_los, 0.0); 

        double psi_los = atan2(-ye, delta_h_);

        double theta_des = pitch_ref + theta_los;
        //anti windup
        if (z_ > -0.5) theta_des = std::max(theta_des, 0.0); 
        
        double psi_des = yaw_ref + psi_los;

        double r_feedforward = normalize_angle(psi_des - last_yaw_ref_) / dt;
        double q_feedforward = normalize_angle(theta_des - last_pitch_ref_) / dt;
        last_pitch_ref_ = theta_des;
        last_yaw_ref_ = psi_des;

        double u_des = u_ref - k_x_ * xe;
        u_des = std::clamp(u_des, -max_u_, max_u_);
        if (u_ref >= 0.0 && u_des < 0.0) {
            u_des = 0.0;
        }
        bool in_deadzone = false;
        if (has_final_goal_) {
            double dxg = x_ - final_goal_.x;
            double dyg = y_ - final_goal_.y;
            double dzg = z_ - final_goal_.z;
            double dist_to_goal = std::sqrt(dxg * dxg + dyg * dyg + dzg * dzg);
            if (dist_to_goal < stop_radius_) {
                in_deadzone = true;
                u_des = 0.0;
                r_feedforward = 0.0;
                q_feedforward = 0.0;
                last_pitch_ref_ = theta_des;
                last_yaw_ref_ = psi_des;
            }
        }
        
        double W = mass_ * gravity_;
        double B = rho_ * volume_ * gravity_;
        double restoring_force_x = (W - B) * std::sin(theta_);
        double restoring_moment_y = cob_z_ * B * std::sin(theta_);
        //disturbance compensation
        double disturbance_force_comp = 0.0;
        if (has_prediction_) {
            double u_c_global = pred_u_;
            double v_c_global = pred_v_;
            double u_curr_body = u_c_global * std::cos(psi_) + v_c_global * std::sin(psi_);
            double u_rel_body = u_ - u_curr_body;
            disturbance_force_comp = d_u_ * u_ - d_u_ * u_rel_body;
        }

        double u_error = u_ - u_des;
        // control law
        double tau_u = - m22_ * v_ * r_ + m33_ * w_ * q_ 
                       - d_u_ * u_des               
                       - disturbance_force_comp     
                       - k_u_ * u_error             
                       - restoring_force_x;
        if (in_deadzone) {
            tau_u = -disturbance_force_comp;
        }
        
        if (z_ > 0.2 && theta_ < -0.1) tau_u = 0.0;
        if (tau_u < 0.0) tau_u = 0.0;
        tau_u = std::clamp(tau_u, -max_tau_u_, max_tau_u_);
        // anti complex derivate
        double theta_error = theta_ - theta_des;
        double alpha_q = q_feedforward - k_theta_ * theta_error;//auxiliary variable
        theta_filter_.update(alpha_q, dt);
        double q_des = theta_filter_.getSpeed();
        double q_dot_des = theta_filter_.getRate();
        double q_error = q_ - q_des;
        
        double tau_q = - d_q_ * q_des + I_yy_ * q_dot_des - k_q_ * q_error - restoring_moment_y;
        tau_q = std::clamp(tau_q, -max_tau_q_, max_tau_q_);
        
        double psi_error = normalize_angle(psi_ - psi_des);
        double alpha_r = r_feedforward - k_psi_ * psi_error;//auxiliary variable
        yaw_filter_.update(alpha_r, dt);
        double r_des = yaw_filter_.getSpeed();
        double r_dot_des = yaw_filter_.getRate();
        double r_error = r_ - r_des;
        
        double tau_r = - d_r_ * r_des + I_zz_ * r_dot_des - k_r_ * r_error;
        tau_r = std::clamp(tau_r, -max_tau_r_, max_tau_r_);
        if (in_deadzone) {
            tau_q = 0.0;
            tau_r = 0.0;
        }

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = tau_u;
        cmd_msg.angular.y = tau_q;
        cmd_msg.angular.z = -tau_r; 
        pub_cmd->publish(cmd_msg);
    }

public:
    SFController(): Node("sf_controller")
    {
        declare_params();
        load_params_and_init(); 
    
        sub_ref_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "reference_state", rclcpp::QoS(10), std::bind(&SFController::ref_callback, this, std::placeholders::_1)
        );
    
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), std::bind(&SFController::odom_callback, this, std::placeholders::_1)
        );

        sub_final_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
            "final_goal", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg){
                final_goal_ = *msg;
                has_final_goal_ = true;
            }
        );
        
        sub_pred_current_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/predicted_current", 10, std::bind(&SFController::prediction_callback, this, std::placeholders::_1)
        );

        pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
        timer_ = this->create_wall_timer(20ms, std::bind(&SFController::control_loop, this));
    
        RCLCPP_INFO(this->get_logger(), "SF Controller (Compensated) Ready.");
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SFController>());
    rclcpp::shutdown();
    return 0;
}
