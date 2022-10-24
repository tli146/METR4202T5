calibration data for ximea 31706251
copy into ~/.ros/camera_info 

to recalibrate run 
$ rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_ros/ximea_31706251/image_raw camera:=/ximea_ros/ximea_31706251
