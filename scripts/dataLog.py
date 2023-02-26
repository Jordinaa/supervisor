#https://github.com/jn89b/utm/blob/noetic/utm/scripts/kf_logger.py


	#---------Logfile Setup-------------#
	# populate the data header, these are just strings, you can name them anything
	myData = ["time","quad x", "quad y", "quad z", "kftag x", "kftag y", 
           "kftag z", "tag x", "tag y", "tag z", "tag roll", "tag pitch",
           "true tag x", "true tag y",
           "pitch", "roll"]

	for kf in kf_name_list:
		myData.append(kf+"_position")
		myData.append(kf+"_error_covar")
		# myData.append(uav_name+"_x")
		# myData.append(uav_name+"_y")
		# myData.append(uav_name+"_z")

	# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
	# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
	# be the correct (it will not be the "real" time
	# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
	# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
	# to include an updated absolute path here to the new directory where you want the files to appear
	fileNameBase = "/home/justin/catkin_ws/src/utm/utm/scripts/logfiles/" + "kf_q" + "_"+ datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + "kf_q" + fileNameSuffix
		num = num + 1

	# now we know we have a unique name, let's open the file, 'a' is append mode, in the unlikely event that we open
	# a file that already exists, this will simply add on to the end of it (rather than destroy or overwrite data)
	myFile = open(fileName, 'a')
	with myFile:
		writer = csv.writer(myFile)
		writer.writerow(myData)

	# get the CPU time at which we started the node, we will use this to subtract off so that our time vector
	# starts near 0
	zero_time = rospy.get_time()

	# this is some ros magic to control the loop timing, you can change this to log data faster/slower as needed
	# note that the IMU publisher publishes data at a specified rate (500Hz) and while this number could be
	# changes, in general, you should keep the loop rate for the logger below the loop rate for the IMU publisher
	rate = rospy.Rate(20) #100 Hz
	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():
			# get the current time and subtract off the zero_time offset
			now = (rospy.get_time()-zero_time)
			# create the data vector which we will write to the file, remember if you change
			# something here, but don't change the header string, your column headers won't
			# match the data
			myData = [now, quad.x, quad.y, quad.z, tagekf.x, tagekf.y, tagekf.z,
             tag.x, tag.y, tag.z, tag.roll, tag.pitch,
			 true_tag.x, true_tag.y, attitude.x_state[2], attitude.y_state[2]]
   
			for kf in kf_est_list:
				myData.append(kf[0].coords)
				myData.append(kf[1].coords)
    
			# stick everything in the file
			myFile = open(fileName, 'a')
			with myFile:
				writer = csv.writer(myFile)
				writer.writerow(myData)

			# this is ros magic, basically just a sleep function with the specified dt
			rate.sleep()