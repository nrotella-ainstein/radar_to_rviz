#!/usr/bin/env python

import rospy
from panda import Panda
from std_msgs.msg import Float64

wheel_speed = 0
WS_FL = 0
WS_FR = 0
WS_RL = 0
WS_RR = 0

KPH_TO_MPS = 0.2778

def car_speed_pub():

    # Create a set up the publisher for car speed:
    pub = rospy.Publisher( 'car_speed', Float64, queue_size=10 )
    rospy.init_node( 'car_speed_pub', anonymous=True )
    pub_rate = rospy.Rate( 10 ) # 10 Hz

    # Interface to Panda:
    panda = Panda()

    # Run endlessly while ROS is running:
    while not rospy.is_shutdown():

        # Get the CAN message from the Panda:
        data = panda.can_recv()
        
	# Find and parse wheel speed data message:
	for i in range( 0, len( data ) ):
	    if data[i][0] == 597:
		wheel_speed = data[i][2]
                
	wheel_speed_b = wheel_speed[0]

	for i in range( 1, len( wheel_speed ) ):
	    wheel_speed_b = ( wheel_speed_b << 8 ) + wheel_speed[i]

	WS_FL = KPH_TO_MPS * wheel_speed[0]
	WS_FR = KPH_TO_MPS * wheel_speed[1]
	WS_RL = KPH_TO_MPS * wheel_speed[2]
	WS_RR = KPH_TO_MPS * wheel_speed[3]
	
	#print "WS_FL = " + str(WS_FL) + "; WS_FR = " + str(WS_FR) + "; WS_RL = " + str(WS_RL) + "; WS_RR = " + str(WS_RR)

        pub.publish( 0.25 * ( WS_FL + WS_FR + WS_RL + WS_RR ) )
        pub_rate.sleep()

if __name__ == '__main__':
    try:
        car_speed_pub()
    except rospy.ROSInterruptException:
        pass
