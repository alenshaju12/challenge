#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class LifecyclePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
LifecyclePublisher()
: rclcpp_lifecycle::LifecycleNode("publisher", rclcpp::NodeOptions().use_global_arguments(false).arguments({"--ros-args", "-r", "__ns:=/demo"}))
  {
    // Declare parameters
    this->declare_parameter<double>("publish_rate_hz", 2.0);
    this->declare_parameter<std::string>("message_text", "hello from lifecycle publisher");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring publisher...");

    publisher_ = this->create_publisher<std_msgs::msg::String>("/demo/chatter", 10);

    double hz = this->get_parameter("publish_rate_hz").as_double();
    std::chrono::milliseconds period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));

    auto publish_msg = [this]() {
      if (!publisher_->is_activated()) {
        return;
      }
      auto msg = std_msgs::msg::String();
      msg.data = this->get_parameter("message_text").as_string();
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    };

    timer_ = this->create_wall_timer(period, publish_msg);
    timer_->cancel(); 

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating publisher...");
    publisher_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating publisher...");
    publisher_->on_deactivate();
    timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up publisher...");
    publisher_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down publisher...");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<LifecyclePublisher>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
