#ifndef FRANKA_ROS2_BRIDGE_HPP
#define FRANKA_ROS2_BRIDGE_HPP

#include <franka/robot.h>
#include <atomic>
#include <mutex>
#include <tbb/concurrent_queue.h>

template <typename T>
class FrankaROS2Bridge {
    using BridgeBuffer = tbb::concurrent_queue<T>;
    using BridgeBufferPtr = std::shared_ptr<BridgeBuffer>;
    using FrankaState = franka::RobotState;

    public:
        using ValueType = T;
        FrankaROS2Bridge() {
            std::cout << "FrankaROS2Bridge constructor" << std::endl;
            bridge_buffer_ = std::make_shared<BridgeBuffer>();
        };

        void setRobotState(const franka::RobotState& robot_state) {
            std::lock_guard<std::mutex> lock(mutex_);
            franka_state_ = robot_state;
        }

        franka::RobotState getRobotState() {
            std::lock_guard<std::mutex> lock(mutex_);
            return franka_state_;
        }

        void pushBridgeBuffer(const T& data) {
            bridge_buffer_->push(data);
        }

        bool popBridgeBuffer(T& data) {
            return bridge_buffer_->try_pop(data);
        }


    private:
        BridgeBufferPtr bridge_buffer_;
        FrankaState franka_state_;
        std::mutex mutex_;

};

#endif