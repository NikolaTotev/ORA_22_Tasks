//System headers
#include <cstdint>

#include "utils/ErrorCode.h"
#include "utils/Log.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);


  // const auto appCfg = generateAppConfig();

  // Application app;
  // const int32_t errCode = app.init(appCfg);
  // if (EXIT_SUCCESS != errCode) {
  //   std::cerr << "initApp() failed" << std::endl;
  //   return EXIT_FAILURE;
  // }

  // app.run();

  std::cout<<"Ooga booga builds"<<std::endl;
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}