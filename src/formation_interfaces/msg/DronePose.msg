# One drone's OptiTrack pose, tagged with its unique ID so it can travel on a
# single shared topic instead of one topic per drone.
int32 drone_id
geometry_msgs/PoseStamped pose
