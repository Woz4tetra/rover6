# Map recording
`roslaunch rover6_config rover6.launch`
`roslaunch rover6_config rplidar.launch`
`roslaunch rover6_config record.launch`

# Map making
`rosbag play --clock [bag name]`
`roslaunch rover6_config gmapping.launch`
`roslaunch map_server map_saver`  # after map is done

# Localization from a bag
`rosbag play --clock [bag name]`
`roslaunch rover6_config amcl.launch map_name:=[map name].yaml from_bag:=true`