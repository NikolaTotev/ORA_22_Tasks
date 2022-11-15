#include <URFinalBridge.h>
#include <std_msgs/msg/string.hpp>
#include "urscript_common/defines/UrScriptTopics.h"
#include <thread>

int32_t URFinalBridge::init()
{

constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);
  rclcpp::PublisherOptions publisherOptions;
  publisherOptions.callback_group = _publishersCallbackGroup;

  _urscriptPublisher = create_publisher<std_msgs::msg::String>(URSCRIPT_TOPIC, qos,
      publisherOptions);

    return EXIT_SUCCESS;
}

URFinalBridge::URFinalBridge() : Node("SOME_NAME")
{
}

void URFinalBridge::run()
{
    std::cout << "Sending Commands" << std::endl;
    sendUrScript();
    using namespace std::literals;
    std::this_thread::sleep_for(1000ms);
    //int i = 0;
    // while(true){
    //     i++;
    // }
}

void URFinalBridge::sendUrScript(){
 std_msgs::msg::String msg;
 std::cout<<"sending movej"<<std::endl;
  msg.data = "def leanForward():\n\tmovej([0.4537856055,-2.00712864,-2.059488517,-0.6457718232,1.570796327,2.024581932], a=5.0, v=1.0)\nend\n";
  _urscriptPublisher->publish(msg);
}