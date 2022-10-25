    #include <std/map.hpp>

    enum State {
        DRIVING,
        READY,
        CALCULATING,
        COLLIDED,
        STOPPED,
        BLOCKED,
        DONE_EXPLORING
    };
    
    enum Event{
        COLLISION,              
    };

    enum Action {
        PROCESS_COLLISION,
        MOVES_ONE_FORWARD,
        TURN_LEFT,
        TRUN_RIGHT,
        CHECK_FRONT,
        CHECK_RIGHT,
        CHECK_LEFT,
        DRIVE_TO_NODE,
        FIND_CLOSEST_UNEXPLORED,
        NOTIFIES_TASK_ORCHESTRATOR,
        FACE_UNEXPLORED_NEIGHBOR,
        DRIVE_UNTIL_COLLISION
    };

    enum ActionResult { 
        COLLISION,
        START_INITIATED,
        COLLISION_PROCESSED,
        ARRIVED_AT_NODE,        
        GET_EMPTY_NODE_LIST,
        MOVES_ONE_FORWARD,
        RIGHT_BORDER_VANISHES,
        FACING_UNXPL_NEIGHBOR,
        RETURNED_TO_START,
        ARRIVED_AT_FULLY_EXPLORED,
        GET_CLOSEST_NODE,
        TURNED_LEFT,
        TURNED_RIGHT,
        
        FRONT_CLEAR,
        FRONT_BLOCKED,
        
        RIGHT_CLEAR,
        RIGHT_BLOCKED,

        LEFT_CLEAR,
        LEFT_BLOCKED,
    };

typedef ActionTransition While;
typedef State Is;
typedef Action And_Does;
typedef ActionResult Happens;
typedef TransitionResult Then;


struct ActionTransition {
    State state;
    ActionResult actionResult;

    ActionTransition(State state, ActionResult result) : state(state), actionResult(result){}
    
};

struct EventBasedTransition {
    State state;
    Event event;
};

struct TransitionResult {
    State state;
    Action action;

    TransitionResult(State state, Action action) : state(state), action(action){}
};


//1. state + event = new state + action 

//2. state + action result = new_state + action



//1. driving + collision -> collided process_collision 
//2. collided + collision_processed -> ready for next move + CHECK_FRONT
//3. ready for next move + front clear -> DRIVING

//3.1. ready for next move + front blocked -> blocked + process_front_block
//4.1  blocked + front_block_processed -> ready for next move + check front
//5.1  ready for next move + front clear -> DRIVING

//1. Driving + tracked border vanished -> stop + check right
//2. stop + right free = ready for next move + turn right
//3. ready for next move + turned right -> DRIVING + start driving






