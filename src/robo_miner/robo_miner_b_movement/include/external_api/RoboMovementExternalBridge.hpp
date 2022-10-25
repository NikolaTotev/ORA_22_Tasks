#include <rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <robo_miner_interfaces/srv/robo_brainz_movement.hpp>
#include <robo_miner_interfaces/srv/robot_move.hpp>

class RoboMovementExternalBridge : public rclcpp::Node
{
public:
    int32_t init();

private:
    void handleMoveService(const std::shared_ptr<RoboBrainzMovement::Request> request, std::shared_ptr<RoboBrainzMovement::Response> response);
    std::shared_ptr<rclcpp::Service<RoboBrainzMovement>> _movementService;
    std::shared_ptr<rclcpp::Client<RobotMove>> _robotMoveClient;
    void callExternalMoveService();
};