#include <cmath>
#include <iostream>
#include <atomic>
#include <array>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <tbb/concurrent_queue.h>

#include "wholebody_control/franka_ros2_bridge.hpp"
#include "examples_common.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using T = std::array<double, 7>;
using Bridge = FrankaROS2Bridge<T>;
using BridgePtr = std::shared_ptr<Bridge>;


class FrankaVelocityControl : public rclcpp::Node {
public:
  FrankaVelocityControl(const std::string& robot_ip, BridgePtr& bridge_ptr)
    : Node("franka_velocity_control"), bridge_ptr_(bridge_ptr) {
    joint_state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), [this]() -> void {
          auto robot_state = bridge_ptr_->getRobotState();
          std_msgs::msg::Float64MultiArray msg;
          for (int i = 0; i<7; i++) {
            msg.data.push_back(robot_state.q[i]);
            msg.data.push_back(robot_state.dq[i]);
          }
          
          joint_state_publisher_->publish(msg);
        });

    joint_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/commands", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() == 7) {
            for (size_t i = 0; i < 7; i++) {
              joint_positions[i] = msg->data[i];
            //   RCLCPP_INFO(this->get_logger(), "Received joint : %f", joint_positions[i]);
            }
            bridge_ptr_->pushBridgeBuffer(joint_positions);
          } else {
            RCLCPP_WARN(this->get_logger(), "Received incorrect joint velocity array size.");
          }
        });
  }
  ~FrankaVelocityControl() {
    t.join();
  }
  std::thread t;

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_publisher_;
  std::atomic<double> time_;
  double time_max_;
  std::shared_ptr<tbb::concurrent_queue<std::array<double, 7>>> vec_;
  BridgePtr bridge_ptr_;
  Bridge::ValueType joint_positions = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // 초기 속도값
  // create timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }


  auto franka_ros2_bridge = BridgePtr(new Bridge());
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FrankaVelocityControl>(argv[1], franka_ros2_bridge);

  franka::Robot robot("172.16.0.1");
  setDefaultBehavior(robot);

  auto t = std::thread([&robot, &franka_ros2_bridge](){
    std::array<double, 7> q_goal = {{0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::Model model = robot.loadModel();
    Bridge::ValueType joint_command = {{-100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0}};

    const std::array<double, 7> k_gains = {{15.0, 15.0, 15.0, 15.0, 15.0, 8.0, 15.0}};
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 20.0, 10.0}};

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques
     {
      auto recived = franka_ros2_bridge->popBridgeBuffer(joint_command);
      franka_ros2_bridge->setRobotState(robot_state);

      if (joint_command[0] <= -100.0) {
        // std::cout << "???" << std::endl;
        return robot_state.tau_J_d;
      }

      std::array<double, 7> coriolis = model.coriolis(robot_state);
      std::array<double, 7> tau_d_calculated;

      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = k_gains[i] * (joint_command[i] - robot_state.q[i]) - d_gains[i] * robot_state.dq[i] + coriolis[i];
      }
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);
      // std::cout << "joint1 vel: " << joint_velocities[0] << std::endl;
        //   franka::JointVelocities velocities = joint_velocities;

        std::cout << tau_d_calculated[0] << std::endl;
      return tau_d_rate_limited;
    });
  });
  
  rclcpp::spin(node);
  t.join();
  

  // node->t.join();
  rclcpp::shutdown();
  

  return 0;
}
