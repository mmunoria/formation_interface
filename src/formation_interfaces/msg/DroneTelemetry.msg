# One drone's live mission status, tagged by name (see
# config/drone_profiles/) rather than DronePose/DroneActive's int32
# drone_id - deliberate divergence: "clearly map to a drone" and "quickly
# varying assignment" argue against a fixed-length-array index as identity.
string drone_name

# Mirrors px4_msgs/VehicleStatus, kept as LOCAL enum copies (not a px4_msgs
# dependency) so this message - and any pure-Python code that reads it -
# stays usable with zero px4_msgs installed, matching the mock-backend goal.
uint8 nav_state
uint8 NAV_STATE_UNKNOWN = 255
uint8 NAV_STATE_OFFBOARD = 14
uint8 NAV_STATE_AUTO_LOITER = 4
uint8 NAV_STATE_AUTO_LAND = 18
uint8 NAV_STATE_AUTO_RTL = 5
uint8 arming_state
uint8 ARMING_STATE_UNKNOWN = 0
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED = 2

bool pre_flight_checks_pass    # sensor handshake
bool gcs_connection_lost       # remote connection
bool failsafe_active
bool link_ok                   # deploy-backend-level liveness (ssh reachable / mock heartbeat fresh)

float32 battery_remaining      # 0..1, -1 if unknown
float32 battery_voltage_v      # -1 if unknown
float32 tracking_error         # m, -1 if unknown

string mission_name            # "" if no mission active for this drone
string mission_phase           # idle|deploying|streaming|holding|landing|terminating|done|error
string error_message           # "" if none
