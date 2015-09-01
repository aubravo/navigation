# navigation

Navigation nodes used for Donaxi a service robot made by UPAEP for the @Home competition.

# Joystick data translation

The joy_translate node works by taking the ROS joy node published data and transforming it into a Twist message (linear and angular speed) under the /cmd_vel topic.

# Move base node

The node in charge of translating the data published on the /cmd_vel topic and transforming it into the rotation speed required by each wheel for the Donaxi robot to move as required by the command velocity. 

This node is alo in charge of the control of the speed by the use of Odometry, thanks to the readings given by encoders, accelerometer, and compass.

This node is currently programmed to work with the Pololu Mini Servo Master (6) for the regulation of each motor rotation speed, and with the Phidgets High Speed Encoders (4) and the Phidgets Spatial (3/3/3).