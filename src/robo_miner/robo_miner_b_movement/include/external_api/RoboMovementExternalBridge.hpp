#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include <robo_miner_interfaces/srv/robo_brainz_movement.hpp>
#include <robo_miner_interfaces/srv/robot_move.hpp>
#include <robo_miner_common/defines/RoboMinerTopics.h>
#include <robo_miner_common/defines/RoboMovementDefines.hpp>

class RoboMovementExternalBridge : public rclcpp::Node
{
public:

    RoboMovementExternalBridge();

    int32_t init();

    void run(); 

private:
    void handleMoveService(const std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Request> request, std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Response> response);
    std::shared_ptr<robo_miner_interfaces::srv::RobotMove::Response> callExternalMoveService(uint8_t dir);
    std::shared_ptr<rclcpp::Service<robo_miner_interfaces::srv::RoboBrainzMovement>> _movementService;
    std::shared_ptr<rclcpp::Client<robo_miner_interfaces::srv::RobotMove>> _robotMoveClient;    
};
