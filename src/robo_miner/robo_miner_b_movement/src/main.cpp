//System headers
#include <cstdint>

#include "utils/ErrorCode.h"
#include "utils/Log.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "external_api/RoboMovementExternalBridge.hpp"

static void runApp(
    const std::shared_ptr<RoboMovementExternalBridge> &node) {
  node->run();
}

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RoboMovementExternalBridge>();
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