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

    // Direct JointState publisher for hand joints (published at fixed rate on arm_joint_states)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_state_publisher_;
    rclcpp::TimerBase::SharedPtr hand_state_timer_;
    void hand_state_timer_callback();

    // Subscriber for UDP hand status feedback
    rclcpp::Subscription<rm_ros_interfaces::msg::Handstatus>::SharedPtr hand_status_subscriber_;

    // Cached hand joint names (from first trajectory goal)
    std::vector<std::string> hand_joint_names_;
    bool hand_names_cached_ = false;

    // Cached hand joint positions in radians (for open-loop or last-known feedback)
    std::array<std::atomic<double>, 6> hand_joint_radians_{};  // order matches hand_joint_names_
    std::mutex hand_mutex_;

    // Hand action callbacks
    rclcpp_action::GoalResponse hand_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse hand_handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void hand_handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void hand_execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // UDP hand status callback
    void hand_state_callback(const rm_ros_interfaces::msg::Handstatus::SharedPtr msg);

    // Hand joint conversion constants
    // HW order: [little(0), ring(1), middle(2), index(3), thumb_flex(4), thumb_rot(5)]
    // Max radians per HW index (from URDF joint limits)
    static constexpr double HW_MAX_RAD[6] = {1.344, 1.344, 1.344, 1.344, 0.5236, 1.246165};

    // Map HW index → feedback array index (matching driver's hand_joint_names config order)
    // Driver config: [thumb_1(rot), thumb_2(flex), index, middle, ring, little]
    // HW[0]=little→fb[5], HW[1]=ring→fb[4], HW[2]=middle→fb[3],
    // HW[3]=index→fb[2], HW[4]=thumb_flex→fb[1], HW[5]=thumb_rot→fb[0]
    static constexpr int HW_TO_FEEDBACK_IDX[6] = {5, 4, 3, 2, 1, 0};

    // Map a joint name (from trajectory) to its HW index by suffix matching
    static int joint_name_to_hw_index(const std::string& name);
};

#endif // Rm_Control_H

