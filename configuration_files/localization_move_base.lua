
include "2d_online_localization.lua"

options.use_odometry = false
options.provide_odom_frame = true
options.odom_frame = "odom" --"odom_combined"

return options
