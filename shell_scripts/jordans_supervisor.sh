#!/bin/sh
echo "running supervisor"

rosrun supervisor assessment.py & sleep 3 & rosrun supervisor fesupervisor.py & wait