#include <external_api/RoboMovementExternalBridge.hpp>

template <typename T>
void waitForService(const T &client)
{
    using namespace std::literals;
    while (!client->wait_for_service(1s))
    {
        std::cout << "Service: [" << client->get_service_name()
                  << "] not available. Waiting for 1s ..." << std::endl;
    }
}

int32_t RoboMovementExternalBridge::init()
{
    using namespace std::placeholders;

    _robotMoveClient = create_client<robo_miner_interfaces::srv::RobotMove>(ROBOT_MOVE_SERVICE);
    waitForService(_robotMoveClient);

    _movementService = create_service<robo_miner_interfaces::srv::RoboBrainzMovement>(PROXY_ROBOT_MOVE_SERVICE,
      std::bind(&RoboMovementExternalBridge::handleMoveService, this,
          _1, _2));
    run();
    return EXIT_SUCCESS;
}

RoboMovementExternalBridge::RoboMovementExternalBridge() : Node("NODE_NAME")
{
}

void RoboMovementExternalBridge::run()
{
    std::cout << "Started movement node." << std::endl;
}

std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> RoboMovementExternalBridge::callExternalMoveService(uint8_t dir)
{
    std::cout<<"Calling external service." << std::endl;
    std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Request> request = std::make_shared<robo_miner_interfaces::srv::RobotMove::Request>();

    request->robot_move_type.move_type = dir;
    
    auto result = _robotMoveClient->async_send_request(request);
    std::cout<<"Awaiting result." << std::endl; //Last line of code that gets called.
    const std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> response = result.get();
    return response;
}

void RoboMovementExternalBridge::handleMoveService(const std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Request> request,
                                                    std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Response> response)
{
    std::cout<<"Move service called" << std::endl;
    auto move_type = request->direction.move_type;

    std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> move_result = callExternalMoveService(move_type);
    response->is_success = move_result->robot_position_response.success;
    response->surrounding_tiles = move_result->robot_position_response.surrounding_tiles;
    std::cout<<"Move service call complete." << std::endl;
}