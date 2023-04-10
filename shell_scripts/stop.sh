#!/bin/bash

# Set the target pane
PANE_ZERO="$SESSION_NAME:0.0"
PANE_TWO="$SESSION_NAME:0.2"
PANE_THREE="$SESSION_NAME:0.3"

# Run the command in the target pane
# tmux send-keys -t $PANE_ZERO C-c Enter 'clear' Enter
tmux send-keys -t $PANE_ZERO C-c Enter 
tmux send-keys -t $PANE_TWO C-c Enter 'clear' Enter
# tmux send-keys -t $PANE_THREE C-c Enter 'clear' Enter