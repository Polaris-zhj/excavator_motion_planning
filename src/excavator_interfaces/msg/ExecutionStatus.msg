uint8 IDLE=0
uint8 VALIDATING=1
uint8 RUNNING=2
uint8 SUCCEEDED=3
uint8 CANCELED=4
uint8 REJECTED=5
uint8 FAILED=6

std_msgs/Header header
uint8 state
string message
float64 elapsed_time
float64 tracking_error_norm
