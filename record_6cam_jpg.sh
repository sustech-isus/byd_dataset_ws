rosbag record -b 4096 --chunksize=1024 --duration=1m /pandar_points /radar_points /radar_tracks /cameras/front/image_color/compressed /cameras/front_left/image_color/compressed /cameras/front_right/image_color/compressed /cameras/rear_left/image_color/compressed /cameras/rear_right/image_color/compressed /cameras/rear/image_color/compressed /infrared_camera/rear/image_color /infrared_camera/rear_left/image_color /infrared_camera/front_left/image_color /infrared_camera/front/image_color /infrared_camera/front_right/image_color /infrared_camera/rear_right/image_color /cameras/driver_camera/image_color/compressed

