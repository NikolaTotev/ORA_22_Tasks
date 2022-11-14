//System headers
#include <cstdint>

#include "utils/ErrorCode.h"
#include "utils/Log.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "external_api/RoboExplorer.hpp"


static void runApp(
    const std::shared_ptr<RoboExplorer> &node) {
  node->run();
}


int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);
  std::cout << "Ooga booga" << std::endl;


  auto node = std::make_shared<RoboExplorer>();
  const int32_t errCode = node->init();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "RoboMovementExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::thread spinThread([&node]() {
    rclcpp::spin(node);
  });

  runApp(node);
  

  rclcpp::shutdown();
  spinThread.join();

  return EXIT_SUCCESS;
}