# Starting RPY command scripts

## QGC
## MAVROS
## PX4 and SITL
## Running Scripts
./QGroundControl.AppImage
roslaunch mavros px4.launch
make px4_sitl_default gazebo_plane
rosrun offboard_py offb_node.py


## running scripts
In px4 terminal
'''
commander takeoff
'''
run script
'''
rosrun offboard_py offb_node.py 
'''

## tmux setup for fourpanes and directories

'''
tmux new -s commandcenter
alt+b %
alt+b "
alt+b left arrow
alt+b %
'''


