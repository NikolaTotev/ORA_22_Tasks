#include <rclcpp/node.hpp>
#include <RoboMovementDefines.hpp>
#include <std/map.hpp>
#include <ExplorerStateMachine.hpp>
#include <ErrorCode.h>
class RoboExploreer : public rclcpp::Node
{

public:
void startExplore();
ErrorCode init();

private:
bool explorationIsComplete;
State currentState;

std::map<ActionTransition,TransitionResult> actionTransitionTable;
std::map<EventBasedTransition, TransitionResult> eventTransitionTable;

void executeAction(Action action);
void transitionToSate(ActionResult actionResult);

ErrorCode setupTransitionTables();



};