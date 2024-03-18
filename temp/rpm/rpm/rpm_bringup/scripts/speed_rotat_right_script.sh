	# !/bin/sh
#programm the inverters to correct configuration
##read paramaters from inverter
###read : vCL_Throttle_enable 1=yes 0=n0
cansend can0 626#40673300
cansend can0 627#40673300
###read :Config of RPDO 1
#timeout 0=  disable timeout check; 1-65535 time in ms
cansend can0 626#40001405
cansend can0 627#40001405

# Send RPDO1 speed
## Speed = 5%
echo "Speed = 5%"
cansend can0 226#97F9
cansend can0 227#6806
cansend can0 626#40663300
cansend can0 627#40663300
sleep 2

## Speed = 10%
echo "Speed = 10%"
cansend can0 226#2FF3
cansend can0 227#D00C
cansend can0 626#40663300
cansend can0 627#40663300
sleep 2

## Speed = 20%
echo "Speed = 20%"
cansend can0 226#5FE6
cansend can0 227#A019
#cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 40%
echo "Speed = 40%"
cansend can0 226#BFCC
cansend can0 227#4033
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 80%
echo "Speed = 80%"
cansend can0 226#7F99
cansend can0 227#8066
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 90%
echo "Speed = 90%"
cansend can0 226#AF8C
cansend can0 227#5070
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 80%
echo "Speed = 80%"
cansend can0 226#7F99
cansend can0 227#8066
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 40%
echo "Speed = 40%"
cansend can0 226#BFCC
cansend can0 227#4033
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 20%
echo "Speed = 20%"
cansend can0 226#5FE6
cansend can0 227#A019
cansend can0 626#40663300
cansend can0 627#40663300
sleep 1

## Speed = 00%
echo "Speed = 0%"
cansend can0 226#0000
cansend can0 227#0000
cansend can0 626#40663300
cansend can0 627#40663300

