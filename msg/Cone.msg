# A single detected cone in the base_link frame.
# x = forward, y = left (ROS REP-103)

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1  # left boundary
uint8 COLOR_YELLOW  = 2  # right boundary
uint8 COLOR_ORANGE  = 3  # start/finish gate

float64 x
float64 y
uint8 color        # see COLOR_* constants above
float64 confidence # 0.0 – 1.0
float64 radius     # fitted base radius in metres
