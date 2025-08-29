#include <chrono>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager()
  : Node("lifecycle_manager")
  {
    declare_parameter("sleep_seconds", 5);
    nodes_ = {"demo/publisher", "demo/subscriber"};
  }

  void run()
  {
    // Ensure all services are available
    for (auto & node_name : nodes_) {
      wait_for_service(node_name, "change_state");
      wait_for_service(node_name, "get_state");
    }

    // Sequence
    configure_all();
    activate_all();

    int sleep_sec = this->get_parameter("sleep_seconds").as_int();
    RCLCPP_INFO(get_logger(), "Sleeping %d seconds...", sleep_sec);
    rclcpp::sleep_for(std::chrono::seconds(sleep_sec));

    transition("demo/publisher", "deactivate");
    RCLCPP_INFO(get_logger(), "Publisher deactivated: Subscriber should not log messages.");
    rclcpp::sleep_for(3s);

    transition("demo/publisher", "activate");
    transition("demo/subscriber", "deactivate");
    RCLCPP_INFO(get_logger(), "Subscriber deactivated: Publisher publishes but not processed.");
    rclcpp::sleep_for(3s);

    transition("demo/subscriber", "activate");
    deactivate_all();
    cleanup_all();
    shutdown_all();
  }

private:
  std::vector<std::string> nodes_;

  void wait_for_service(const std::string & node, const std::string & service)
  {
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + node + "/change_state");
    while (!client->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for %s/%s service...", node.c_str(), service.c_str());
    }
  }

  void call_change_state(const std::string & node, uint8_t transition_id)
  {
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + node + "/change_state");
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    RCLCPP_INFO(this->get_logger(), "Transitioned %s with id %u", node.c_str(), transition_id);
  }

  void configure_all()
  {
    for (auto & node : nodes_) call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }

  void activate_all()
  {
    for (auto & node : nodes_) call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  void deactivate_all()
  {
    for (auto & node : nodes_) call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  void cleanup_all()
  {
    for (auto & node : nodes_) call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  }

void shutdown_all()
{
  for (auto & node : nodes_) {
    call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
  }
}


  void transition(const std::string & node, const std::string & action)
  {
    if (action == "configure")
      call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    else if (action == "activate")
      call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    else if (action == "deactivate")
      call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    else if (action == "cleanup")
      call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    else if (action == "shutdown")
      call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto manager = std::make_shared<LifecycleManager>();
  manager->run();
  rclcpp::shutdown();
  return 0;
}
