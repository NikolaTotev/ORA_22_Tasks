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
 
  msg.data = "def leanForward():\n\tmovel([0.491,-0.134,0.1,2.224,-2.224,0], a=1.0, v=0.42)\nend\n";
  _urscriptPublisher->publish(msg);
}