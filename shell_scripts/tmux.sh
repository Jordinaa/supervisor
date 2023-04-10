#!/bin/bash
# this runs tmux and opens up all programs for running the PX4 SITL
# set the session name
SESSION_NAME="PX4"


# start a new tmux session
tmux new-session -d -s $SESSION_NAME

# split the window into panes
tmux split-window -h -t $SESSION_NAME 
tmux split-window -v -t $SESSION_NAME
tmux select-pane -L
tmux split-window -v -t $SESSION_NAME

# run commands in each pane
tmux send-keys -t $SESSION_NAME:0.0 'roscd supervisor; code .; cd ~' Enter
tmux send-keys -t $SESSION_NAME:0.1 'cd; cd PX4-Autopilot; make px4_sitl gazebo_plane' Enter
tmux send-keys -t $SESSION_NAME:0.2 'cd ~' Enter
tmux send-keys -t $SESSION_NAME:0.3 'cd ~' Enter

# open terminal tabs
gnome-terminal --tab --active --title="MAVROS" -- bash -c "cd; roslaunch mavros px4.launch; exec bash" Enter
gnome-terminal --tab --active --title="QGC" -- bash -c "cd; ./QGroundControl.AppImage; exec bash"
gnome-terminal --tab --active --title="Misc" -- bash -c "cd; exec bash"

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME

export SESSION_NAME="PX4"
