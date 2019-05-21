#!/usr/bin/
#This code was written by Caleb G. Teague in 2019


from MinIMU_v5_pi import MinIMU_v5_pi


def main():
	#Setup the MinIMU_v5_pi
	IMU = MinIMU_v5_pi()
	
	#Initiate tracking of Yaw on the IMU
	IMU.trackYaw()

	while True: #Main loop             
		time.sleep(1)
		yaw = IMU.prevYaw[0]
		print yaw


if __name__ == "__main__":
	main()


