# 실행 상태
uint8 STATE_RECEIVED=0
uint8 STATE_WAITING_FOR_EXECUTION=1
uint8 STATE_VALIDATING=2
uint8 STATE_HANDOFF=3
uint8 STATE_EXECUTING=4
uint8 STATE_COMPLETED=5
uint8 STATE_FAILED=6
uint8 STATE_CANCELLED=7

std_msgs/Header header
string scan_id
string execution_id
string workflow_id
uint8 state
uint32 move_index
uint32 total_moves
string current_move
uint32 step_index
uint32 total_steps
string hand_id
string primitive_id
string error_code
string message
