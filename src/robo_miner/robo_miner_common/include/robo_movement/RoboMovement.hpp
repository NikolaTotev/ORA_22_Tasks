#include <rclcpp/node.hpp>
#include <RoboMovementDefines.hpp>

class RoboMovement : public rclcpp::Node
{

public:
NodeType checkDirection(Direction directionToCheck);
MoveResult move(Direction moveDirection);

private:

};
