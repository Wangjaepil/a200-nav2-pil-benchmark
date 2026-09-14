# Shadow-mode decision produced for each received candidate velocity command.
# A negative TTC/CPA/clearance means that the value is not available.

uint8 ACTION_NO_DATA=0
uint8 ACTION_PASS=1
uint8 ACTION_SLOW=2
uint8 ACTION_STOP=3

uint8 MODEL_NONE=0
uint8 MODEL_CONSTANT_VELOCITY=1
uint8 MODEL_IMM=2

uint32 NO_TRACK_ID=4294967295

std_msgs/Header header
bool valid
uint8 action
uint8 prediction_model
uint32 track_id
float64 ttc_sec
float64 time_to_closest_approach_sec
float64 min_clearance_m
float64 speed_scale
float64 obstacle_speed_mps
float64 obstacle_data_age_sec
string reason
