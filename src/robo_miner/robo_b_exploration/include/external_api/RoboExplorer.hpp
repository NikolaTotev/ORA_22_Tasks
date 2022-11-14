#include <defines/ExplorerStateMachine.hpp>
#include <robo_miner_interfaces/srv/robo_brainz_movement.hpp>
#include <robo_miner_common/defines/RoboMinerTopics.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>

struct RobotState{
    State state;
    uint8_t direction;
    uint8_t currentTile;
    uint8_t surroundings[3];
    uint8_t rear;
};

class RoboExplorer : public rclcpp::Node
{

public:

RoboExplorer();
void startExplore();
int32_t init();
void run();

private:
bool explorationIsComplete;
RobotState currentState;

std::map<ActionTransition,TransitionResult> actionTransitionTable;

void executeAction(Action action);
void transitionToSateWith(ActionResult actionResult);

int32_t setupTransitionTables();

robo_miner_interfaces::srv::RoboBrainzMovement::Response& move(uint8_t movementDirection);
std::shared_ptr<rclcpp::Client<robo_miner_interfaces::srv::RoboBrainzMovement>> _movementClient;

ActionResult aMoveOneForward();
ActionResult aProcessCollision();
ActionResult aTurnLeft();
ActionResult aTurnRight();
ActionResult aCheckFront();
ActionResult aCheckRight();
ActionResult aCheckLeft();
ActionResult aDriveToNode();
ActionResult aFindClosestUnexplored();
ActionResult aNofityTaskOrchestrator();
ActionResult aFaceUnexploredNeighbor();
ActionResult aDriveUntilCollision();

};