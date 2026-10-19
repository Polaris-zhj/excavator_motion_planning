# Unified execution state constants. Must match tramcar::planning::ExecutionState.
uint8 IDLE=0
uint8 VALIDATING=1
uint8 READY=2
uint8 RUNNING=3
uint8 PAUSED=4
uint8 COMPLETED=5
uint8 CANCELLED=6
uint8 SAFETY_STOP=7
uint8 ERROR=8

std_msgs/Header header
uint8 state
string message
float64 elapsed_time
float64 duration
float64 progress
float64 tracking_error_norm
