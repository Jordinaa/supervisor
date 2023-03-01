# ros

## publisher
lists ros topics
rostopic list

gives you type of message it needs to recieve
rostopic info /pkgName


whats inside of type and what args and hwo it takes
rosmsg show typefrompkgname


Looking at rostopic data
rostopic echo /mavros/setpoint_raw/attitude

finding topic
$ rostopic list | grep counter