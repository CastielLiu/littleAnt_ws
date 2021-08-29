#VCU_8 0x151

std_msgs/Header header

uint8 GEAR_DRIVE   = 11
uint8 GEAR_NEUTRAL = 12
uint8 GEAR_REVERSE = 13

uint8 act_gear
float32 accel_pedal_position
uint8 brake_pedal
bool accel_pedal_position_valid
bool act_gear_valid
bool brake_pedal_valid
bool vehicle_ready
bool driverless_mode


