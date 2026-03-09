#ifndef RM_CONTROL_H
#define RM_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <iostream>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

//RM Robot msg
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/liftheight.hpp"
#include "rm_ros_interfaces/msg/udpliftstate.hpp"
#include "rm_ros_interfaces/msg/handangle.hpp"
#include "rm_ros_interfaces/msg/handstatus.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
//#include "rm_ros_interfaces/msg/jointpos75.hpp"

/* 使用变长数组 */
#include <vector>
#include <algorithm>
#include <atomic>
#include <mutex>

using namespace std;

class Rm_Control : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit Rm_Control(std::string name);
    ~Rm_Control(){}

    void timer_callback();

private:
    rm_ros_interfaces::msg::Jointpos joint_msg;
    // rm_ros_interfaces::msg::Jointpos75 joint7_msg;
    int arm_type_ = 75;
    bool follow_ = false;
    // 实例化样条
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // 声明话题发布者
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher;
    // rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos75>::SharedPtr joint_pos_publisher_75;

    //声明话题订阅者
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Move_Stop_Cmd;

    rclcpp::TimerBase::SharedPtr State_Timer;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void get_move_stop_callback(std_msgs::msg::Empty::SharedPtr msg);

    // ======================== Pole (Lift) Support ========================
    // Pole action server (separate from arm action server)
    std::string pole_action_name_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr pole_action_server_;

    // Publisher for lift height command (fire-and-forget to driver)
    rclcpp::Publisher<rm_ros_interfaces::msg::Liftheight>::SharedPtr lift_height_publisher_;

    // Subscriber for UDP lift state feedback
    rclcpp::Subscription<rm_ros_interfaces::msg::Udpliftstate>::SharedPtr lift_state_subscriber_;

    // Cached lift state from UDP feedback
    std::atomic<double> current_lift_height_hw_{0.0};  // in hardware units
    std::mutex pole_mutex_;

    // Pole action callbacks
    rclcpp_action::GoalResponse pole_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse pole_handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void pole_handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void pole_execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // UDP lift state callback
    void lift_state_callback(const rm_ros_interfaces::msg::Udpliftstate::SharedPtr msg);

    // Unit conversion constants
    static constexpr double MOVEIT_TO_HW = 2.0 / 3.0;   // MoveIt meters -> mm -> HW: * 1000 * (2/3)
    static constexpr double HW_TO_MOVEIT = 1.5;          // HW -> mm -> MoveIt meters: * 1.5 / 1000

    // ======================== Hand (Dexterous Hand) Support ========================
    static constexpr int HAND_DOF = 6;
    bool enable_hand_ = false;
    std::string hand_action_name_;
    std::string hand_feedback_mode_;  // "open_loop" or "udp"

    // Hand action server (separate from arm and pole action servers)
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr hand_action_server_;

    // Publisher for hand angle command (fire-and-forget to driver)
    rclcpp::Publisher<rm_ros_interfaces::msg::Handangle>::SharedPtr hand_angle_publisher_;

    // Publisher for open-loop feedback (commanded radians → driver joint_states)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_feedback_publisher_;

    // Subscriber for UDP hand status feedback
    rclcpp::Subscription<rm_ros_interfaces::msg::Handstatus>::SharedPtr hand_status_subscriber_;

    // Cached hand joint positions in radians (for open-loop or last-known feedback)
    std::array<std::atomic<double>, 6> hand_joint_radians_{};  // MoveIt order
    std::mutex hand_mutex_;

    // Hand action callbacks
    rclcpp_action::GoalResponse hand_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse hand_handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void hand_handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void hand_execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // UDP hand status callback
    void hand_state_callback(const rm_ros_interfaces::msg::Handstatus::SharedPtr msg);

    // Hand joint conversion constants
    // MoveIt order: [thumb1_flex, thumb2_rot, index, middle, ring, little]
    // HW order:     [little, ring, middle, index, thumb_flex, thumb_rot]
    // max_rad per MoveIt joint index (from XACRO)
    static constexpr double HAND_MAX_RAD[6] = {1.344, 1.246165, 1.344, 1.344, 1.344, 1.344};
    // Mapping: MoveIt index -> HW index
    static constexpr int HAND_MOVEIT_TO_HW[6] = {4, 5, 3, 2, 1, 0};
    // Mapping: HW index -> MoveIt index
    static constexpr int HAND_HW_TO_MOVEIT[6] = {5, 4, 3, 2, 0, 1};
};

#endif // Rm_Control_H

