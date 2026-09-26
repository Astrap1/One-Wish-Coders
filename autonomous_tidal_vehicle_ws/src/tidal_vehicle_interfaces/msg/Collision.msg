# A detected collision (Version 3). Published by the vehicle's collision monitor
# on /vehicle/collision when a contact sensor or the IMU jolt check fires.
std_msgs/Header header
# contact | imu
string source
# hull | skirt | track_left | track_right | unknown (imu)
string part
# front | rear | left | right, from the contact point (or jolt direction)
string side
# What was hit, if Gazebo knows it (model name), else empty
string other
# Impact strength: horizontal jolt in m/s^2 (imu) or contact depth in m (contact)
float32 strength
