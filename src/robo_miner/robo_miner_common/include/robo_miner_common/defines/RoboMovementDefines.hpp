enum class NodeType {
  CYAN,
  PURPLE,
  BLUE,
  GREEN,
  RED
};

enum class Direction{
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD
};

struct MoveResult{
    bool is_success;
    int8_t in_front;
    int8_t in_left;
    int8_t in_right;
};