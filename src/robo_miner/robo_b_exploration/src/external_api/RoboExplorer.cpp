#include "external_api/RoboExplorer.hpp"

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

RoboExplorer::RoboExplorer() : Node("explorer_node")
{
}

int32_t RoboExplorer::init()
{
    setupTransitionTables();
    currentState.state = Is::READY;
    _movementClient = create_client<robo_miner_interfaces::srv::RoboBrainzMovement>(PROXY_ROBOT_MOVE_SERVICE);
    waitForService(_movementClient);
    return EXIT_SUCCESS;
}

void RoboExplorer::run()
{
    std::cout << "Starting exploration" << std::endl;
    transitionToSateWith(Happens::START_INITIATED);
}

int32_t RoboExplorer::setupTransitionTables()
{
    // 1. state + event = new state + action

    // 2. state + action result = new_state + action
    std::cout << "Setting up transition tables." << std::endl;

    actionTransitionTable = {
        {While(Is::READY, Happens::START_INITIATED), Then(Is::DRIVING, And_Does::DRIVE_UNTIL_COLLISION)},

        {While(Is::DRIVING, Happens::COLLISION), Then(Is::COLLIDED, And_Does::PROCESS_COLLISION)},
        {While(Is::COLLIDED, Happens::COLLISION_PROCESSED), Then(Is::READY, And_Does::TURN_LEFT)},

        //This line is for testing.
        {While(Is::COLLIDED, Happens::GET_EMPTY_NODE_LIST), Then(Is::DONE_EXPLORING, And_Does::NOTIFIES_TASK_ORCHESTRATOR)},
        
        {While(Is::READY, Happens::TURNED_LEFT), Then(Is::READY, And_Does::CHECK_FRONT)},
        {While(Is::READY, Happens::FRONT_CLEAR), Then(Is::DRIVING, And_Does::MOVE_ONE_FORWARD)},
        {While(Is::DRIVING, Happens::MOVED_ONE_FORWARD), Then(Is::STOPPED, And_Does::CHECK_RIGHT)},

        {While(Is::STOPPED, Happens::RIGHT_CLEAR), Then(Is::READY, And_Does::TURN_RIGHT)},
        {While(Is::READY, Happens::TURNED_RIGHT), Then(Is::DRIVING, And_Does::MOVE_ONE_FORWARD)},
        {While(Is::STOPPED, Happens::RIGHT_BLOCKED), Then(Is::DRIVING, And_Does::MOVE_ONE_FORWARD)},

        {While(Is::READY, Happens::FRONT_BLOCKED), Then(Is::BLOCKED, And_Does::TURN_LEFT)},
        {While(Is::BLOCKED, Happens::TURNED_LEFT), Then(Is::READY, And_Does::CHECK_FRONT)},

        {While(Is::DRIVING, Happens::ARRIVED_AT_FULLY_EXPLORED), Then(Is::CALCULATING, And_Does::FIND_CLOSEST_UNEXPLORED)},
        {While(Is::DRIVING, Happens::RETURNED_TO_START), Then(Is::CALCULATING, And_Does::FIND_CLOSEST_UNEXPLORED)},
        {While(Is::CALCULATING, Happens::GET_EMPTY_NODE_LIST), Then(Is::DONE_EXPLORING, And_Does::NOTIFIES_TASK_ORCHESTRATOR)},
        {While(Is::CALCULATING, Happens::GET_CLOSEST_NODE), Then(Is::DRIVING, And_Does::DRIVE_TO_NODE)},
        {While(Is::DRIVING, Happens::ARRIVED_AT_NODE), Then(Is::STOPPED, And_Does::FACE_UNEXPLORED_NEIGHBOR)},
        {While(Is::STOPPED, Happens::FACING_UNXPL_NEIGHBOR), Then(Is::DRIVING, And_Does::DRIVE_UNTIL_COLLISION)}};

    return 22;
}

void RoboExplorer::executeAction(Action action)
{
    ActionResult result = ActionResult::ARRIVED_AT_FULLY_EXPLORED;

    switch (action)
    {
    case Action::PROCESS_COLLISION:
    std::cout << "Executing process collision" << std::endl;
        result = aProcessCollision();
        break;
    case Action::MOVE_ONE_FORWARD:
        result = aMoveOneForward();
        break;
    case Action::TURN_LEFT:
        break;
    case Action::TURN_RIGHT:
        break;
    case Action::CHECK_FRONT:
        break;
    case Action::CHECK_LEFT:
        break;
    case Action::CHECK_RIGHT:
        break;
    case Action::DRIVE_TO_NODE:
        break;
    case Action::FIND_CLOSEST_UNEXPLORED:
        break;
    case Action::NOTIFIES_TASK_ORCHESTRATOR:
        
        std::cout << "Ooga booga done" << std::endl;
        return;

        break;
    case Action::FACE_UNEXPLORED_NEIGHBOR:
        break;
    case Action::DRIVE_UNTIL_COLLISION:
        std::cout << "Driving until collision" << std::endl;
        result =  aDriveUntilCollision();

        break;
    default:
        break;
    }

    transitionToSateWith(result);
}

void RoboExplorer::transitionToSateWith(ActionResult result)
{
   std::cout<<"Attempting transition" << std::endl;
   std::cout << result << std::endl;
   std::cout << "Current state:"<<std::endl;
   std::cout << currentState.state << std::endl;

    if(actionTransitionTable.count(While(currentState.state, result))){
        auto mapValue = actionTransitionTable.find(While(currentState.state, result));
        currentState.state = mapValue->second.first;
        Action actionToExecute = mapValue->second.second;
        std::cout<<"Result:"<<std::endl;
        std::cout<<result<<std::endl;
        std::cout<<"New State:"<<std::endl;
        std::cout<<currentState.state<<std::endl;
        std::cout<<"Action to execute:"<<std::endl;
        std::cout<<actionToExecute<<std::endl;
        executeAction(actionToExecute); 
    }
}

ActionResult RoboExplorer::aMoveOneForward()
{
    bool frontClear = currentState.surroundings[0] != 88 && currentState.surroundings[0] != 35;

    if (frontClear)
    {
        auto &response = move(0);
        // Shift tiles before reassigning surroundings array;
        currentState.rear = currentState.currentTile;
        currentState.currentTile = currentState.surroundings[1];

        // Reassign surroundings array with new values;
        currentState.surroundings[0] = response.surrounding_tiles[0];
        currentState.surroundings[1] = response.surrounding_tiles[1];
        currentState.surroundings[2] = response.surrounding_tiles[2];
        return ActionResult::MOVED_ONE_FORWARD;
    }

    return ActionResult::COLLISION;
}

ActionResult RoboExplorer::aProcessCollision(){return ActionResult::GET_EMPTY_NODE_LIST; }

ActionResult RoboExplorer::aTurnLeft(){return ActionResult::NORES; }
ActionResult RoboExplorer::aTurnRight(){return ActionResult::NORES; }
ActionResult RoboExplorer::aCheckFront(){return ActionResult::NORES; }
ActionResult RoboExplorer::aCheckRight(){return ActionResult::NORES; }
ActionResult RoboExplorer::aCheckLeft(){return ActionResult::NORES; }
ActionResult RoboExplorer::aDriveToNode(){return ActionResult::NORES; }
ActionResult RoboExplorer::aFindClosestUnexplored(){return ActionResult::NORES; }
ActionResult RoboExplorer::aNofityTaskOrchestrator(){return ActionResult::NORES; }
ActionResult RoboExplorer::aFaceUnexploredNeighbor(){return ActionResult::NORES; }

ActionResult RoboExplorer::aDriveUntilCollision()
{

    while (currentState.surroundings[0] != 88 && currentState.surroundings[0] != 35)
    {
        std::cout << "Move 1 forward" << std::endl;

        auto &response = move(0);
        // Shift tiles before reassigning surroundings array;
        //currentState.rear = currentState.currentTile;
        //currentState.currentTile = currentState.surroundings[1];

        // Reassign surroundings array with new values;
        currentState.surroundings[0] = response.surrounding_tiles[0];
        currentState.surroundings[1] = response.surrounding_tiles[1];
        currentState.surroundings[2] = response.surrounding_tiles[2];
    }

    return ActionResult::COLLISION;
}

robo_miner_interfaces::srv::RoboBrainzMovement::Response& RoboExplorer::move(uint8_t movementDirection){
auto request = std::make_shared<robo_miner_interfaces::srv::RoboBrainzMovement::Request>();

    request->direction.move_type = movementDirection;

    auto result = _movementClient->async_send_request(request);
    std::shared_ptr<robo_miner_interfaces::srv::RoboBrainzMovement::Response> response = result.get();
    std::cout<<"Something" << std::endl;
    std::cout << response->is_success<<std::endl;
    return * response;
}
