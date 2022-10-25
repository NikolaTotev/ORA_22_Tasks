#include <RoboExplorer.hpp>
#include <std/map.hpp>
#include <std/pair.hpp>
#include <ExplorerStateMachine.hpp>

ErrorCode RoboExploreer::init()
{
    currentState = Is::Ready;    
}


ErrorCode RoboExploreer::setupTransitionTables()
{
    //1. state + event = new state + action 

    //2. state + action result = new_state + action

    actionTransitionTable = {
        {While(Is::READY, Happens::START_INITIATED), Then(Is::DRIVING, And_Does::DRIVE_UNTIL_COLLISION)},

        {While(Is::DRIVING, Happens::COLLISION), Then(Is::COLLIDED, And_Does::PROCESS_COLLISION)},
        {While(Is::COLLIDED, Happens::COLLISION_PROCESSED), Then(Is::READY, And_Does::TURN_LEFT)},
        {While(Is::READY, Happens::TURNED_LEFT), Then(Is::READY, And_Does::CHECK_FRONT)},
        {While(Is::READY, Happens::FRONT_CLEAR), Then(Is::DRIVING, And_Does::MOVES_ONE_FORWARD)},
        {While(Is::DRIVING, Happens::MOVES_ONE_FORWARD), Then(Is::STOPPED, And_Does::CHECK_RIGHT)},

        {While(Is::STOPPED, Happens::RIGHT_CLEAR), Then(Is::READY, And_Does::TRUN_RIGHT)},
        {While(Is::READY, Happens::TURNED_RIGHT), Then(Is::DRIVING, And_Does::MOVES_ONE_FORWARD)},
        {While(Is::STOPPED, Happens::RIGHT_BLOCKED), Then(Is::DRIVING, And_Does::MOVES_ONE_FORWARD)},
        
        {While(Is::READY, Happens::FRONT_BLOCKED), Then(Is::BLOCKED, And_Does::TURN_LEFT)},
        {While(Is::BLOCKED, Happens::TURNED_LEFT), Then(Is::READY, And_Does::CHECK_FRONT)},

        {While(Is::DRIVING, Happens::ARRIVED_AT_FULLY_EXPLORED), Then(Is::CALCULATING, And_Does::FIND_CLOSEST_UNEXPLORED)},
        {While(Is::DRIVING, Happens::RETURNED_TO_START), Then(Is::CALCULATING, And_Does::FIND_CLOSEST_UNEXPLORED)},
        {While(Is::CALCULATING, Happens::GET_EMPTY_NODE_LIST), Then(Is::DONE_EXPLORING, And_Does::NOTIFIES_TASK_ORCHESTRATOR)},
        {While(Is::CALCULATING, Happens::GET_CLOSEST_NODE), Then(Is::DRIVING, And_Does::DRIVE_TO_NODE)},
        {While(Is::DRIVING, Happens::ARRIVED_AT_NODE), Then(Is::STOPPED, And_Does::FACE_UNEXPLORED_NEIGHBOR)},
        {While(Is::STOPPED, Happens::FACING_UNXPL_NEIGHBOR), Then(Is::DRIVING, And_Does::DRIVE_UNTIL_COLLISION)}
    };
}

void RoboExploreer::executeAction(Action action)
{
    ActionResult result;

    switch (action)
    {
    case Action::MOVES_ONE_FORWARD:
        break;
    case Action::PROCESS_COLLISION:
        break;
    default:
        break;
    }

    transitionToSate(result);
}

void RoboExploreer::transitionToSate(ActionResult result){
}
