#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class LifecycleSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public: 
LifecycleSubscriber()
: rclcpp_lifecycle::LifecycleNode(
      "subscriber", 
      rclcpp::NodeOptions()
          .use_global_arguments(false)
          .arguments({"--ros-args", "-r", "__ns:=/demo"})
    ),
  active_(false)
{}


protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring subscriber...");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/demo/chatter", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (active_) {
          RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        }
      }
    );
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating subscriber...");
    active_ = true;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating subscriber...");
    active_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up subscriber...");
    subscription_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down subscriber...");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  bool active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<LifecycleSubscriber>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
