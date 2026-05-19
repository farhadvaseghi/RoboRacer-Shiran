# A single detected object in the base_link frame.
# x = forward, y = left (ROS REP-103)
#
# Replaces the legacy Cone.msg — same fields, more generic name since the
# track now uses tube walls + opponent rather than cones.

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1  # left / inner boundary
uint8 COLOR_YELLOW  = 2  # right / outer boundary
uint8 COLOR_ORANGE  = 3  # start/finish gate
uint8 COLOR_RED     = 4  # opponent / dynamic obstacle

float64 x
float64 y
uint8 color        # see COLOR_* constants above
float64 confidence # 0.0 – 1.0

# Shape (from PCA on LIDAR clusters, or bounding box on camera blobs).
# For long objects (walls): length > width, yaw = wall direction.
# For roughly square objects (opponent): length ≈ width, yaw = opp heading.
# Camera-only detections set yaw = 0.0 (no orientation info from a single bbox).
float64 length     # principal axis extent (m)
float64 width      # minor axis extent (m)
float64 yaw        # orientation of principal axis (rad). 0.0 if not computed.

# Legacy lossy size summary, kept for backward compatibility with cone-era code.
# Equals max(length, width) / 2.
float64 radius
