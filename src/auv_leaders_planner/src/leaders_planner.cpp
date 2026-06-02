#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <Eigen/Dense> 
#include"rclcpp/rclcpp.hpp"
#include"nav_msgs/msg/odometry.hpp"
#include"auv_leaders_planner/msg/aux_var.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;
class leaders_node :public rclcpp::Node
{
private:
// 1.Data structure
struct AgentInfo
{
Eigen::Vector3d z;//local formation center
Eigen::Vector3d s;//reference velocity
Eigen::Vector3d rho;//formation error
bool received = false;
rclcpp::Time last_seen;
};
// 2.Personal id & variables
int my_id_;
std::vector<int> target_neighbors_ids_;//neighbors

//real state：position & velocities
Eigen::Vector3d real_pos_ = Eigen::Vector3d::Zero();
Eigen::Vector3d real_vel_ = Eigen::Vector3d::Zero();
bool has_real_odom_ = false;
bool first_run_ = true;

Eigen::Vector3d output_pos_ = Eigen::Vector3d::Zero();
double output_u_ = 0.0;
double output_pitch_ = 0.0;
double output_yaw_ = 0.0;

//reference variables（due to the dynamics limits from ECA A9）
Eigen::Vector3d ref_var_z_  = Eigen::Vector3d::Zero();
Eigen::Vector3d ref_var_s_ = Eigen::Vector3d::Zero();
Eigen::Vector3d ref_var_rho_ =Eigen::Vector3d::Zero();

//Mission center send or not
Eigen::Vector3d global_goal_;
Eigen::Vector3d received_global_goal_;   
bool has_received_goal_ = false;         
bool goal_published_ = false;
//offset shape
std::map<int, Eigen::Vector3d> formation_offsets_map_;
Eigen::Vector3d final_goal_; 

//control parameters
double k1_,k2_,k3_,k4_,k5_,k6_,beta_;
//physical limits
double max_u_, max_pitch_rate_,max_yaw_rate_;
double max_rho_mag_;

//deadzone brake
bool is_in_deadzone_ = false; 
double dz_enter_radius_;      
double dz_exit_radius_;       
double dz_brake_gain_;        
double dz_damping_gain_; 
double dz_enter_speed_;
double dz_enter_s_norm_;
double dz_enter_rho_norm_;

//neighbor lable
std::map<int,AgentInfo> neighbors_;

//port
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_odom_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_ref_;
rclcpp::Publisher<auv_leaders_planner::msg::AuxVar>::SharedPtr pub_broadcast_;
std::vector<rclcpp::Subscription<auv_leaders_planner::msg::AuxVar>::SharedPtr> sub_neighbors_;
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_global_goal_;//if is leader
rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_global_goal_;//if is follower
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_local_final_goal_;//To controller
rclcpp::TimerBase::SharedPtr timer_;
public:
leaders_node() :Node("leaders_node"){
    declare_params();
    load_params_and_init();
    //initial communication
    init_communication();
}
private:
void declare_params(){
    // Id & neighbors
    this->declare_parameter("agent_id",0);
    this->declare_parameter("neighbors",std::vector<int64_t>{});//不确定大小先这样写
    // Control gain
    this->declare_parameter("controller_gains.k_1", 3.0);
    this->declare_parameter("controller_gains.k2_list", std::vector<double>{1.0, 1.0, 1.0});
    this->declare_parameter("controller_gains.k_3", 2.5);
    this->declare_parameter("controller_gains.k_4", 10.0);
    this->declare_parameter("controller_gains.k_5", 5.0);
    this->declare_parameter("controller_gains.k_6", 1.0);
    this->declare_parameter("controller_gains.beta", 1.0);
    // Limits
    this->declare_parameter("dynamics_limits.max_thrust", 80.0);
    this->declare_parameter("dynamics_limits.max_torque", 15.0);
    this->declare_parameter("dynamics_limits.max_surge_speed", 1.5);
    this->declare_parameter("dynamics_limits.max_yaw_rate", 0.5);
    this->declare_parameter("dynamics_limits.max_pitch_rate", 0.3);
    this->declare_parameter("dynamics_limits.rho_limit", 5.0); 
    // Formation
    this->declare_parameter("formation_offsets.leader_0", std::vector<double>{10.0, 0.0, 10.0});
    this->declare_parameter("formation_offsets.leader_1", std::vector<double>{-10.0, 10.0, -10.0});
    this->declare_parameter("formation_offsets.leader_2", std::vector<double>{-10.0, -10.0, -10.0});
    this->declare_parameter("formation_offsets.global_goal", std::vector<double>{100.0, 50.0, -30.0});
    // deadzone
    this->declare_parameter("deadzone.enter_radius", 1.0);
    this->declare_parameter("deadzone.exit_radius", 0.5);
    this->declare_parameter("deadzone.brake_gain", 40.0);
    this->declare_parameter("deadzone.damping_gain", 15.0);
    this->declare_parameter("deadzone.enter_speed", 0.1);
    this->declare_parameter("deadzone.enter_s_norm", 0.2);
    this->declare_parameter("deadzone.enter_rho_norm", 0.5);
}

    void load_params_and_init(){
        //1.id & neighbors
        my_id_ = this->get_parameter("agent_id").as_int();
        auto nb_vec = this->get_parameter("neighbors").as_integer_array();
        for(auto id: nb_vec) target_neighbors_ids_.push_back((int)id);

        RCLCPP_INFO(this->get_logger(),"第%d的通信中继AUV启动，正在监听%ld个队友",my_id_,target_neighbors_ids_.size());

        //2.Control gains
        k1_ = this->get_parameter("controller_gains.k_1").as_double();
        k3_ = this->get_parameter("controller_gains.k_3").as_double();
        k4_ = this->get_parameter("controller_gains.k_4").as_double();
        k5_ = this->get_parameter("controller_gains.k_5").as_double();
        k6_ = this->get_parameter("controller_gains.k_6").as_double();
        beta_ = this->get_parameter("controller_gains.beta").as_double();
        std::vector<double> k2_list = this->get_parameter("controller_gains.k2_list").as_double_array();
        if (my_id_ >= 0 && my_id_ < (int)k2_list.size()) {
            k2_ = k2_list[my_id_];
        } else {
        k2_ = k2_list.empty() ? 1.0 : k2_list[0];
        RCLCPP_WARN(this->get_logger(), "ID %d 超出 k2_list 范围，使用默认值", my_id_);
        }

        //3.Limits
        max_u_ = this->get_parameter("dynamics_limits.max_surge_speed").as_double();
        max_pitch_rate_ = this->get_parameter("dynamics_limits.max_pitch_rate").as_double();
        max_yaw_rate_ = this->get_parameter("dynamics_limits.max_yaw_rate").as_double();
        max_rho_mag_ = this->get_parameter("dynamics_limits.rho_limit").as_double();

        //4.Targets
        auto goal_vec = this->get_parameter("formation_offsets.global_goal").as_double_array();
        if (goal_vec.size() == 3) global_goal_ = Eigen::Vector3d(goal_vec[0], goal_vec[1], goal_vec[2]);

        for(int i=0; i<3; ++i){
            std::string param_name = "formation_offsets.leader_" + std::to_string(i);
            try{
                auto vec = this->get_parameter(param_name).as_double_array();
                if(vec.size() == 3){
                    formation_offsets_map_[i] = Eigen::Vector3d(vec[0], vec[1], vec[2]);
                }
            }
            catch(...) {
                formation_offsets_map_[i] = Eigen::Vector3d::Zero();
            }
        }
        dz_enter_radius_ = this->get_parameter("deadzone.enter_radius").as_double();
        dz_exit_radius_ = this->get_parameter("deadzone.exit_radius").as_double();
        dz_brake_gain_ = this->get_parameter("deadzone.brake_gain").as_double();
        dz_damping_gain_ = this->get_parameter("deadzone.damping_gain").as_double();
        dz_enter_speed_ = this->get_parameter("deadzone.enter_speed").as_double();
        dz_enter_s_norm_ = this->get_parameter("deadzone.enter_s_norm").as_double();
        dz_enter_rho_norm_ = this->get_parameter("deadzone.enter_rho_norm").as_double();
}

void init_communication(){
    //1.Get local odom
    sub_local_odom_ = this->create_subscription<nav_msgs::msg::Odometry>
    ("odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        real_pos_.x() = msg->pose.pose.position.x;
        real_pos_.y() = msg->pose.pose.position.y;
        real_pos_.z() = msg->pose.pose.position.z;
        real_vel_.x() = msg->twist.twist.linear.x;
        real_vel_.y() = msg->twist.twist.linear.y;
        real_vel_.z() = msg->twist.twist.linear.z;
        // for braking
        if(!has_real_odom_){
            ref_var_z_ = real_pos_ - formation_offsets_map_[my_id_];
            output_yaw_ = 0.0;
            output_pitch_ = 0.0;
            has_real_odom_ =true;
            RCLCPP_INFO(this->get_logger(),"虚拟点初始化成功[%f, %f, %f]", 
                    ref_var_z_.x(), ref_var_z_.y(), ref_var_z_.z());
        }
    });
    

    // 2.publish reference state
    pub_local_ref_ = this->create_publisher<nav_msgs::msg::Odometry>("reference_state",10);

    // 3.communication with neighbors
    std::string broadcast_topic = "/swarm_comms/auv" + std::to_string(my_id_);
    pub_broadcast_ = this->create_publisher<auv_leaders_planner::msg::AuxVar>(broadcast_topic,10);

    // 4.subscribe neighbors' variables
    for(int nb_id : target_neighbors_ids_) {
        std::string topic = "/swarm_comms/auv" + std::to_string(nb_id);
        auto cb = [this,nb_id](const auv_leaders_planner::msg::AuxVar::SharedPtr msg) {
            neighbors_[nb_id].z = Eigen::Vector3d(msg->z.x, msg->z.y, msg->z.z);
            neighbors_[nb_id].s = Eigen::Vector3d(msg->s.x, msg->s.y, msg->s.z);
            neighbors_[nb_id].rho = Eigen::Vector3d(msg->rho.x, msg->rho.y, msg->rho.z);

            neighbors_[nb_id].received = true;
            neighbors_[nb_id].last_seen = this->now();
        };
        sub_neighbors_.push_back(this->create_subscription<auv_leaders_planner::msg::AuxVar>(topic,10,cb));
    } 

    rclcpp::QoS qos_setting(1);
    qos_setting.transient_local();   
    qos_setting.reliable();

    // 1. Publisher（only using by leader）
        pub_global_goal_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/swarm_global_goal", qos_setting);

    // 1.5 Publish final goal for local controller
    pub_local_final_goal_ = this->create_publisher<geometry_msgs::msg::Point>("final_goal", 10);

    // 2. Subscriber（only using by leader）
    sub_global_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/swarm_global_goal", qos_setting,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            received_global_goal_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
            has_received_goal_ = true;
            RCLCPP_INFO(this->get_logger(), "收到全局目标: [%.2f, %.2f, %.2f]",
                        received_global_goal_.x(), received_global_goal_.y(), received_global_goal_.z());
        });

    // 5.circulation loop
    timer_ = this->create_wall_timer(20ms,std::bind(&leaders_node::control_loop,this));
    
}
// kinematic limits
Eigen::Vector3d kinematic_limits(Eigen::Vector3d &s_curr, const Eigen::Vector3d &s_dot_raw, double dt){
    Eigen::Vector3d s_next_ideal = s_curr + s_dot_raw * dt;
    Eigen::Vector3d s_prev = s_curr;

    double u_next = s_next_ideal.norm();

    if(u_next > max_u_) s_next_ideal = s_next_ideal.normalized() * max_u_;
    u_next = s_next_ideal.norm();

    if (u_next < 0.05){
        s_curr = s_next_ideal;
        return (s_curr - s_prev) / dt;
    }

    //desired angle
    double pitch_next = std::atan2(s_next_ideal.z(), std::hypot(s_next_ideal.x(), s_next_ideal.y()));
    double yaw_next = std::atan2(s_next_ideal.y(), s_next_ideal.x());

    //current angles
    double pitch_curr = std::atan2(s_curr.z(), std::hypot(s_curr.x(), s_curr.y()));
    double yaw_curr = std::atan2(s_curr.y(), s_curr.x());
    if(s_curr.norm() < 0.05) {pitch_curr = pitch_next; yaw_curr = yaw_next;}

    if(u_next > max_u_) u_next = max_u_;

    double diff_pitch = pitch_next - pitch_curr;
    while (diff_pitch > M_PI) diff_pitch -=2*M_PI;
    while (diff_pitch < -M_PI) diff_pitch +=2*M_PI;
    double max_p_delta = max_pitch_rate_*dt;
    if(std::abs(diff_pitch) > max_p_delta) pitch_next = pitch_curr + (diff_pitch > 0 ? 1.0 : -1.0)*max_p_delta;

    //pitch physical limits
    double max_abs_pitch = 80.0 * M_PI / 180.0;
    if (pitch_next > max_abs_pitch) pitch_next = max_abs_pitch;
    if (pitch_next < -max_abs_pitch) pitch_next = -max_abs_pitch;

    // yaw speed limits
    double diff_yaw = yaw_next - yaw_curr;
    while (diff_yaw > M_PI) diff_yaw -=2*M_PI;
    while (diff_yaw < -M_PI) diff_yaw +=2*M_PI;
    double max_y_delta = max_yaw_rate_*dt;
    if(std::abs(diff_yaw) > max_y_delta) yaw_next = yaw_curr + (diff_yaw > 0 ? 1.0 : -1.0)*max_y_delta;

    // A tricky for variable "s"
    s_curr.x() = u_next *cos(pitch_next) * cos(yaw_next);
    s_curr.y() = u_next *cos(pitch_next) * sin(yaw_next);
    s_curr.z() = u_next *sin(pitch_next);

    return (s_curr - s_prev) / dt;
}

void control_loop(){
    if(!has_real_odom_) return;
    double dt =0.02;

    if (my_id_ == 0 && !goal_published_) {
        geometry_msgs::msg::Point goal_msg;
        goal_msg.x = global_goal_.x();
        goal_msg.y = global_goal_.y();
        goal_msg.z = global_goal_.z();

        pub_global_goal_->publish(goal_msg);
        goal_published_ = true;

        RCLCPP_INFO(this->get_logger(), "Leader 已广播全局目标一次: [%.2f, %.2f, %.2f]",
                    global_goal_.x(), global_goal_.y(), global_goal_.z());
    }

    //1.cooperated items' calculation
    Eigen::Vector3d coupling_z = Eigen::Vector3d::Zero();
    Eigen::Vector3d coupling_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d coupling_rho = Eigen::Vector3d::Zero();
    for(int nb_id : target_neighbors_ids_){
        if(!neighbors_[nb_id].received) continue;
        
        Eigen::Vector3d z_j = neighbors_[nb_id].z;
        Eigen::Vector3d s_j = neighbors_[nb_id].s;
        Eigen::Vector3d rho_j = neighbors_[nb_id].rho;

        coupling_z += (ref_var_z_ - z_j);
        coupling_s += (ref_var_s_ - s_j);
        coupling_rho += (ref_var_rho_ - rho_j);
    }
    // 2.local calculcation
    Eigen::Vector3d effective_goal = global_goal_;

    if (my_id_ != 0) {  // follower
        if (has_received_goal_) {
            effective_goal = received_global_goal_;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Follower %d 尚未收到全局目标，暂停前进", my_id_);
            effective_goal = formation_offsets_map_[my_id_]; 
        }
    }

    Eigen::Vector3d local_tracking = ref_var_z_ + formation_offsets_map_[my_id_] - real_pos_;
    Eigen::Vector3d gloabl_tracking = k2_*(ref_var_z_ - effective_goal);
    Eigen::Vector3d s_dot_raw = -k5_*ref_var_s_ - k3_*local_tracking - k4_*gloabl_tracking 
    - coupling_z - beta_*coupling_rho - k6_*ref_var_rho_;
    Eigen::Vector3d rho_dot_raw = coupling_z + beta_*coupling_s;
    //pre-windup for rho
    ref_var_rho_ +=rho_dot_raw*dt;

    if (ref_var_rho_.norm() > max_rho_mag_)
    ref_var_rho_ = ref_var_rho_.normalized() * max_rho_mag_;

    // Anti-Windup for Rho
     Eigen::Vector3d s_dot_limited = kinematic_limits(ref_var_s_, s_dot_raw, dt);

    //3.variables' update
    ref_var_s_ += s_dot_limited * dt;
    ref_var_z_ += ref_var_s_ * dt;

    output_u_ = ref_var_s_.norm();
    if (output_u_ > 0.05){
        output_pitch_ = -std::atan2(ref_var_s_.z(), std::hypot(ref_var_s_.x(), ref_var_s_.y()));
        output_yaw_ = std::atan2(ref_var_s_.y(), ref_var_s_.x());
    }
    
    output_pos_ = ref_var_z_ + formation_offsets_map_[my_id_];

    bool can_calc_deadzone = true;
    if (my_id_ == 0) {
        final_goal_ = global_goal_ + formation_offsets_map_[my_id_];
    } else {
        if (has_received_goal_) {
            final_goal_ = received_global_goal_ + formation_offsets_map_[my_id_];
        } else {
            can_calc_deadzone = false;
        }
    }

    if (can_calc_deadzone) {
        double goal_error_dist = (real_pos_ - final_goal_).norm();

        if (!is_in_deadzone_
            && goal_error_dist < dz_enter_radius_
            && real_vel_.norm() < dz_enter_speed_
            && ref_var_s_.norm() < dz_enter_s_norm_
            && ref_var_rho_.norm() < dz_enter_rho_norm_) {
            is_in_deadzone_ = true;
            RCLCPP_INFO(this->get_logger(), "ID%d 接近最终目标，进入死区刹车 | 剩余距离：%.2fm",
                        my_id_, goal_error_dist);
        } else if (is_in_deadzone_ && goal_error_dist > dz_exit_radius_ * 1.5) {
            is_in_deadzone_ = false;
            RCLCPP_INFO(this->get_logger(), "ID%d 远离最终目标，退出死区 | 当前距离：%.2fm",
                        my_id_, goal_error_dist);
        }

        if (is_in_deadzone_) {
            output_u_ = 0.0;
            ref_var_s_.setZero(); 
            ref_var_z_ = final_goal_ - formation_offsets_map_[my_id_];
            output_pitch_ = 0.0;
            output_yaw_ = 0.0;
            output_pos_ = final_goal_;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "ID%d 死区中：剩余距离%.2fm | 强刹车+姿态阻尼",
                                my_id_, goal_error_dist);
        }
    }
    publish_data();
}

void publish_data(){
    // 1.Broadcast auxiliary variables
    auv_leaders_planner::msg::AuxVar aux_msg;
    aux_msg.agent_id = my_id_;
    aux_msg.z.x = ref_var_z_.x(); aux_msg.z.y = ref_var_z_.y(); aux_msg.z.z = ref_var_z_.z(); 
    aux_msg.s.x = ref_var_s_.x(); aux_msg.s.y = ref_var_s_.y(); aux_msg.s.z = ref_var_s_.z(); 
    aux_msg.rho.x = ref_var_rho_.x(); aux_msg.rho.y = ref_var_rho_.y(); aux_msg.rho.z = ref_var_rho_.z(); 

    pub_broadcast_ -> publish(aux_msg);

    //2.Send messages to local controller
    nav_msgs::msg::Odometry ref_msg;
    ref_msg.header.stamp = this->now();
    ref_msg.header.frame_id = "odom";

    ref_msg.pose.pose.position.x = output_pos_.x();
    ref_msg.pose.pose.position.y = output_pos_.y();
    ref_msg.pose.pose.position.z = output_pos_.z();

    ref_msg.twist.twist.linear.x = output_u_;
    //Turn to quadrupt
    double cy = cos(output_yaw_ * 0.5);
    double sy = sin(output_yaw_ * 0.5);
    double cp = cos(output_pitch_ * 0.5);
    double sp = sin(output_pitch_ * 0.5);
    double cr = 1.0; double sr = 0.0; //roll is ignored

    ref_msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    ref_msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    ref_msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    ref_msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    pub_local_ref_->publish(ref_msg);

    // 3. Send final goal
    geometry_msgs::msg::Point final_goal_msg;
    final_goal_msg.x = final_goal_.x();
    final_goal_msg.y = final_goal_.y();
    final_goal_msg.z = final_goal_.z();
    pub_local_final_goal_->publish(final_goal_msg);
}
};
int main(int argc, char **argv)
{
rclcpp::init(argc,argv);
rclcpp::spin(std::make_shared<leaders_node>());
rclcpp::shutdown();
return 0;
}
