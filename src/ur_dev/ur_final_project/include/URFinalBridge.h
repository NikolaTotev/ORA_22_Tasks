#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>

class URFinalBridge : public rclcpp::Node
{
public:

    URFinalBridge();

    int32_t init();

    void run(); 

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _urscriptPublisher;
  const rclcpp::CallbackGroup::SharedPtr _publishersCallbackGroup =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
void sendUrScript();

};
