#!/bin/bash

# Set the target pane
PANE_ZERO="$SESSION_NAME:0.0"
PANE_TWO="$SESSION_NAME:0.2"
PANE_THREE="$SESSION_NAME:0.3"

# Run the command in the target pane
sleep 2
tmux send-keys -t $PANE_TWO 'clear; rosrun supervisor assessment.py' Enter
sleep 2
tmux send-keys -t $PANE_ZERO 'clear; rosrun supervisor fesupervisor.py' Enter
# tmux send-keys -t $PANE_THREE 'clear; rosrun supervisor visualizer.py' Enter
