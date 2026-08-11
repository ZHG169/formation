builtin_interfaces/Time stamp 
builtin_interfaces/Time execute_at

uint8 leader_id
uint32 leader_generation
uint32 sequence

uint8 command
uint8 HOLD=0
uint8 MOVE=1
uint8 SET_YAW=2
uint8 MISSION_COMPLETE=3
uint8 EMERGENCY_STOP=4

float32 velocity_east
float32 velocity_north
float32 velocity_up

float32 yaw_enu
float32 yaw_rate
float32 duration

string formation_type
float32 spacing
