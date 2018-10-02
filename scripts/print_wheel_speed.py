from panda import Panda

panda = Panda()

wheel_speed = 0
WS_FL = 0
WS_FR = 0
WS_RL = 0
WS_RR = 0

while True:
	# get CAN data
	data = panda.can_recv()
	
	# find wheel speed data
	for i in range(0,len(data)):
		if data[i][0] == 597:
			wheel_speed = data[i][2]

	wheel_speed_b = wheel_speed[0]

	for i in range(1,len(wheel_speed)):
		wheel_speed_b = (wheel_speed_b << 8) + wheel_speed[i]


	WS_FL = wheel_speed[0]
	WS_FR = wheel_speed[1]
	WS_RL = wheel_speed[2]
	WS_RR = wheel_speed[3]
	
	print "WS_FL = " + str(WS_FL) + "; WS_FR = " + str(WS_FR) + "; WS_RL = " + str(WS_RL) + "; WS_RR = " + str(WS_RR)
