#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
double arm_state[4];
double hand_state;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MCU_Publisher : public rclcpp::Node
{
  public:
  
    MCU_Publisher()
    : Node("mcu_pub")
    {
        publisher_arm = this->create_publisher<std_msgs::msg::UInt64>("arm_joint_pos", 10);
        publisher_hand = this->create_publisher<std_msgs::msg::Float64>("hand_joint_pos", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&MCU_Publisher::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
        1ms, std::bind(&MCU_Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        int decimalPlaces = 3;

        u_int64_t dir = 0;
        u_int64_t pos = 0;
        for (int i = 0; i<4; ++i){
            if (arm_state[i] < 0.0) {
                dir += (u_int64_t)pow(10, i+16);
            }
            u_int64_t temp = (u_int64_t)(abs(arm_state[i]) * pow(10, decimalPlaces));
            pos = pos + temp * (u_int64_t)pow(10, i*4);
        }
        pos += dir;

        auto message_arm = std_msgs::msg::UInt64();
        message_arm.data = pos;
        publisher_arm->publish(message_arm);

        auto message_hand = std_msgs::msg::Float64(); 
        message_hand.data = 100+(int)(100*hand_state/1.48);
        publisher_hand->publish(message_hand);
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        for (int i = 0; i<4; ++i){
            arm_state[i] = msg->position[i];
        }
        hand_state = msg->position[4];
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_arm;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_hand;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCU_Publisher>());
  rclcpp::shutdown();
  return 0;
}