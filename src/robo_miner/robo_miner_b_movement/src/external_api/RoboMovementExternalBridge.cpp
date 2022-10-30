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

    return EXIT_SUCCESS;
}

RoboMovementExternalBridge::RoboMovementExternalBridge() : Node("NODE_NAME")
{
}

void RoboMovementExternalBridge::run()
{
    
}

std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> RoboMovementExternalBridge::callExternalMoveService(uint8_t dir)
{
    auto request = std::make_shared<robo_miner_interfaces::srv::RobotMove::Request>();

    request->robot_move_type.move_type = dir;
    
    auto result = _robotMoveClient->async_send_request(request);
    const std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> response = result.get();
    return response;
}

void RoboMovementExternalBridge::handleMoveService([[maybe_unused]] const std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Request> request,
                                                   [[maybe_unused]] std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Response> response)
{
    auto move_type = request->direction.move_type;

    std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> move_result = callExternalMoveService(move_type);
    response->is_success = move_result->robot_position_response.success;
    response->surrounding_tiles = move_result->robot_position_response.surrounding_tiles;
}