builtin_interfaces/Time stamp
uint8 mode
uint8 gait_type
float32 body_height
float32 foot_raise_height
uint8 speed_level
bool is_moving
float32 vx
float32 vy
float32 vyaw
float32 progress

uint8 MODE_IDLE=0
uint8 MODE_BALANCE_STAND=1
uint8 MODE_POSE=2
uint8 MODE_LOCOMOTION=3
uint8 MODE_LIE_DOWN=5
uint8 MODE_JOINT_LOCK=6
uint8 MODE_DAMPING=7
uint8 MODE_RECOVERY_STAND=8
uint8 MODE_SIT=10

uint8 GAIT_IDLE=0
uint8 GAIT_TROT=1
uint8 GAIT_RUN=2
uint8 GAIT_CLIMB_STAIR=3
uint8 GAIT_FORWARD_DOWN_STAIR=4
