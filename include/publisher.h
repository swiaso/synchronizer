#include <chrono>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <random>
using namespace std;

using namespace std::chrono_literals;

namespace synchronizer
{
class Publisher : public rclcpp::Node
{
public:
  Publisher(string topic_name, int real_period_, bool msg_period_bias_flag, double period_factor)
  : Node(topic_name.c_str()), real_period_(real_period_), count_(0), thread_created_(false), msg_period_bias_flag_(msg_period_bias_flag), period_bias_total_(0.0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(topic_name.c_str(), 10);

    RCLCPP_INFO(this->get_logger(), "Period of '%s': '%ld'", topic_name.c_str(), real_period_);

    period_bias_upper_ = (period_factor - 1) * real_period_;

    period_ = 1ms * real_period_;

    Thread = new std::thread(&Publisher::timer_callback,this);
  }

private:
  void thread_create()
  {
    if(!thread_created_)
    {
      thread_created_ = true;
      Thread = new std::thread(&Publisher::timer_callback,this);
      // Thread->join();
    }
  }

  void timer_callback()
  { 
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine e(seed);
    uniform_int_distribution<unsigned> bias(0, period_bias_upper_); // bias for T
    uniform_int_distribution<unsigned> bias_flag(0, 1); // bias or not

    while(rclcpp::ok())
    {
      int period_bias = 0;
      if(msg_period_bias_flag_)
      {
        int flag = bias_flag(e);
        int b = bias(e);
        period_bias = flag * b;

        period_bias_total_ += period_bias;
      }
      
      rclcpp::sleep_for(std::chrono::milliseconds(real_period_ + period_bias));
      rclcpp::Time now = this->now();
      std::string symbol = std::to_string(count_);
      count_++;
      
      if(count_ > 10)
        publisher(symbol, now);
    }
  }

  void publisher(std::string symbol,rclcpp::Time now)
  {
    auto message=sensor_msgs::msg::JointState();
    message.name.push_back("pub1:"+symbol);
    message.header.stamp = now;

    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::chrono::milliseconds period_;
  int real_period_;
  size_t count_;
  bool thread_created_;
  bool msg_period_bias_flag_;

  double period_bias_upper_;
  double period_bias_total_;

  rclcpp::Time timer_basetime_;
  std::thread* Thread;

  rclcpp::Duration PeriodBase = rclcpp::Duration(1, 0); //1s
};

}