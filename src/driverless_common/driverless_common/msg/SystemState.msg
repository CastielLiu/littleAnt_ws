std_msgs/Header header

uint8   STATE_IDLE=0
uint8   STATE_DRIVE=1
uint8   STATE_REVERSE=2
uint8   STATE_STOP=3

uint8   state
string  task_state
float64 position_x
float64 position_y
float64 yaw

float32 lateral_error     #m left(-) right(+)
float32 yaw_error         #deg  ccw(-) cw(+)

float32 command_speed     #km/h
float32 vehicle_speed     #km/h
float32 roadwheel_angle   #deg  ccw(-) cw(+)

float32 nearest_object_distance


