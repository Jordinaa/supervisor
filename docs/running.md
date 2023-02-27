# Starting RPY command scripts

## QGC
'''
./QGroundControl.AppImage
'''

## MAVROS
'''
roslaunch mavros px4.launch
'''

## PX4 and SITL
'''
make px4_sitl_default gazebo_plane
'''

# Running Scripts
'''
rosrun offboard_py offb_node.py 
'''


## running scripts
In px4 terminal
'''
commander takeoff
'''
run script
'''
rosrun offboard_py offb_node.py 
'''
