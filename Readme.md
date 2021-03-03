# Arduino Obstacle Avoiding Robot Car with MPU6050 IMU
Obstracle avoiding robot car using an MPU6050 IMU to detect obstracle in from of the vehicle.
Build with FreeRtos and controlled by two tasks __distanceTask__ and __driveTask__. The __distanceTask__ checks for obstacles with the MPU6050 by detect acceleration and angular velocity 
exceeds of threshold limits from the impact with an obstacle.

When the __imuTask__ detects an obstacle it notify and unblock the the __driveTask__ and then blocks and wait for a notification from the __driveTask__ to unblock.

The __driveTask__ runs as a state machine and go FORWARD until an obstacle is detected (blocks in FORWARD state until notified from distanceTask that obstacle is ahead) then change to REVERSE state and reverse for about 1 second and enter TURN state. Do a short turn in TURN state and return to FORWARD state and notify and unblock imuTask. The TURN and FORWARD states are hold for a second +/- some random ms.

### Hardware list:

Arduino	UNO R3

4x DC Motor Tt 130Motor	DC3V-6V DC Gear Motor

Arduino sensor shield v5.0

L298 Dual full bridge driver

MPU6050 IMU

Generic robot car frame

Powered	by 2x18650 4200 mAh 3.7V li-ion
