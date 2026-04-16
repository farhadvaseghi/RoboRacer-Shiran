# A single detected cone in the vehicle's base_link frame.
# x/y are in metres; x=forward, y=left.

float64 x
float64 y
float64 radius      # approximate cone radius (m)
float64 confidence  # detection confidence [0.0, 1.0]
uint8   color       # one of the COLOR_* constants below

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1
uint8 COLOR_YELLOW  = 2
uint8 COLOR_ORANGE  = 3
