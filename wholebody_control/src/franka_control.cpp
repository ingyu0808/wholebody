// Description: This is a simple example of a Franka Control Node using ROS2.

#include <cmath>
#include <iostream>
#include <atomic>
#include <array>

#include <franka/exception.h>
#include <franka/robot.h>
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
        "/joint_velocity_cmd", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() == 7) {
            for (size_t i = 0; i < 7; i++) {
              joint_velocities_[i] = msg->data[i];
              RCLCPP_INFO(this->get_logger(), "Joint[%d] velocity: %f", i, joint_velocities_[i]);
            }

            bridge_ptr_->pushBridgeBuffer(joint_velocities_);
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
  Bridge::ValueType joint_velocities_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // 초기 속도값
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
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    Bridge::ValueType joint_velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    robot.control([&franka_ros2_bridge, &joint_velocities](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
      auto recived = franka_ros2_bridge->popBridgeBuffer(joint_velocities);
      franka_ros2_bridge->setRobotState(robot_state);
      // std::cout << "joint1 vel: " << joint_velocities[0] << std::endl;
      franka::JointVelocities velocities = joint_velocities;
      return velocities;
    });
  });
  
  rclcpp::spin(node);
  t.join();
  

  // node->t.join();
  rclcpp::shutdown();
  

  return 0;
}
