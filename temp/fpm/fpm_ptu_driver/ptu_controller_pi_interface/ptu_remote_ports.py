"""
The purpose of this software is to enable a temporary operation of the PTU-Controller (Adafruit) with the PTU
and will later be splaced by ROS-communication (client-service).
The setup here is to evaluate a digital 24V-Input (provided by the UR-Robot) and send the derived signals via USB-serial
communication to the controller.
In order to have a minimal diagnosis, in parallel, a LED at the Raspberry-PI resp. Janztec is activated.
There are four digital inputs available, that are directly "translated" into different actions on the PTU. The naming
of the digital signals on the JANZTEC deosn't follow the automation standards beginning with 0. They start with number 1.
Changing of the turning direction isn's taken. into account
DI1: On/Off --> Activation of PUT, analog output is set to Min-value
DI2: RampON --> Start ramping by changing the analog output
DI3/DI4: Defining ramp-up speed
         DI3 and DI4 off: Jump to Max-value
         Remaining switching combinations of DI3 and DI4: defining different ramp up speeds.   

Robert Bosch GmbH
Martin Dieterle - CR/APT5
2022-11-04

Due to continued problems with communication (after <1h), modification of port control (initially for testing)
Changed to opening ports on demand in contrary to leave port open as with original solution.
Port is openede with switching ON/OFF-Bit (both directions) and closed after switching OFF.
This is the crucial bit, no port activity with ramping or ramp speed bits!

"""

import os
import json
import RPi.GPIO as GPIO
import time
import serial
import glob

# Scan for connected devices
def ptu_port_ident():
    # Port list in order to identify connected PTU-controller
    # PTU controller is Adafruit and returns an 'ACM' within its TTY-definition
    ports = glob.glob('/dev/tty[A-Za-z]*')
    for port in ports:
        # Leave loop, when port of suited type found
        if 'ACM' in port:
	    #print('Scanned port tty:', port)
            ptu_port = port
            break
    else:
        ptu_port = None        
        print('No connected Adafruit-device identified')        
    return ptu_port

ptu_port = ptu_port_ident()
print('Selected port for PTU:', ptu_port)

#Init SERIAL INTERFACE VIA USB
# serial on PI
# import serial
# -----------------------------------------------------------------------------
# configure the serial connections (the parameters differs on the device you 
# are connected to, e.g. USB-COM may change depending on connected items)
ser = serial.Serial(
    port= ptu_port,	# use of detected port containing substring "ACM"
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff= False  
)
if not ser.isOpen():
    ser.open()
ser.write(b'USB_Id\n')
# Read response from PTU-Control after pausing in order to include PTU-response
# (Reading of complete buffer)
time.sleep(0.2)
print('Feedback PTU-Control:', str(ser.read(ser.inWaiting())))
ser.close()             # Changed 2022-11-23


# From example 
#Code Part 3: Button Input
#Let's add some user input! Save the following to a file (such as button.py).

"""
From JANZTEC-Dokumentation):
IO  GPIO    Pin
DI1 GPIO6  (Pin 31)
DI2 GPIO13 (Pin 33)
DI3 GPIO16 (Pin 36)
DI4 GPIO19 (Pin 35)
DO1 GPIO23 (Pin 16)
DO2 GPIO22 (Pin 15)
DO3 GPIO27 (Pin 13)
DO4 GPIO18 (Pin 12)

LED:
GREEN LED: GPIO5 (Pin 29)
RED   LED  GPIO12 (Pin 32)
"""

# Pins definitions (not clear, wheterh to use GPIO-number or Pin-number
ptu_ctrl_robot = 6      # DI1
ptu_ctrl_ramp = 13      # DI2
ptu_ctrl_r2 = 16        # DI3
ptu_ctrl_r3 = 19        # DI4
ptu_ctrl_feedback = 23  # DO1, Return DI1 to Robot 
ptu_ramp_feedback = 22  # DO2, Return DI2 to Robot 
led_green = 5           # Green LED
led_red = 12            # Red LED

# Deactivation of warnings from GPIO
GPIO.setwarnings(False)
# Set up pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(ptu_ctrl_robot, GPIO.IN)
GPIO.setup(ptu_ctrl_ramp, GPIO.IN)
GPIO.setup(ptu_ctrl_r2, GPIO.IN)
GPIO.setup(ptu_ctrl_r3, GPIO.IN)
GPIO.setup(ptu_ctrl_feedback, GPIO.OUT)
GPIO.setup(ptu_ramp_feedback, GPIO.OUT)
GPIO.setup(led_green, GPIO.OUT)
GPIO.setup(led_red, GPIO.OUT)

# Init bit for evaluation of input state
ptu_ctrl_robot_last = GPIO.input(ptu_ctrl_robot)
ptu_ctrl_ramp_state_last = GPIO.input(ptu_ctrl_ramp)

# Init-values for ramp and start 
sp_rpm = sp_rpm_ramp_min = 45
sp_rpm_ramp_max = 70

# Lists for values to print out (diagnosis)
time_list =  []
rpm_list = []
print_once = False

# For testing - assign time value of the moment
last_time = now = time.time()
comm_check_time = 30

try:
    # If button is pushed, light up LED
    while True:
        # Evaluate change of input values (Write ON OFF only on change, not permanently)
        # LED is controlled in parallel in order to indicate desired state
        # Assigning signal only once in order to avoid different values during execution of code
        # Feedback to robot in order to check communication
        ptu_ctrl_state = GPIO.input(ptu_ctrl_robot)
        if ptu_ctrl_state:
            GPIO.output(ptu_ctrl_feedback, GPIO.HIGH)  
        else:
            GPIO.output(ptu_ctrl_feedback, GPIO.LOW)  
        # Assign decrement incerement inputs for consistency reasons (no change during code execution)
        ptu_ctrl_ramp_state = GPIO.input(ptu_ctrl_ramp)
        if ptu_ctrl_ramp_state:
            GPIO.output(ptu_ramp_feedback, GPIO.HIGH)  
        else:
            GPIO.output(ptu_ramp_feedback, GPIO.LOW)  

        # For testing stability of communication
        # Regular sending of string to PTU
        # In a second step, the port should be re-established in case it's not working 
        # (e.g. had been disconnected)
        if not ptu_ctrl_state and not ptu_ctrl_ramp_state:
            now = time.time()
            if now - last_time > comm_check_time:    # Value in seconds
                try:
                    if not ser.isOpen():                # Changed/added 2022-11-23
                        ser.open()
                    #print('Aktuelle Zeit', now, last_time, now - last_time)
                    ser.write(b'USB_Id\n')
                    # Read response from PTU-Control after pausing in order to include PTU-response
                    # (Reading of complete buffer)
                    time.sleep(0.2)
                    print('Comm.check: Feedback PTU-Control: after', str(ser.read(ser.inWaiting())),now - last_time)
                    last_time = now
                    ser.close()             # Changed/added 2022-11-23
                    # Communication check:
                    print('Communication successful at:', now)
                except:
                    print('Communication NOT successful at:', now)
        else:
            # Reset timer with activation of PTU in order to start after activation from scratch
            last_time = now

# 14.11.2022        
#        EXTENSION OF BLINKING LED  
#        # LED blinks, as long as the PTU isn't switched ON          
#        now = time.time()
#        delta = now - lastchange
#            if now - lastchange > 0.5:
#                ledstate = not ledstate
#                lastchange = now
        if ptu_ctrl_state != ptu_ctrl_robot_last:
            if not ser.isOpen():                # Changed/added 2022-11-23
                ser.open()
            # Change of port control, port is only opened on demand 
            # Input is TRUE and last cycle was FALSE --> Switch ON PTU
            if ptu_ctrl_state:
                print("PTU On")
                ser.write(b'D5_On\n')    
                # Define RPM-Setpoint for starting, Changed from rpm_start to rpm_min  2022-11-15
                ser.write(b'A0=' + str(sp_rpm_ramp_min).encode('utf-8') +'\n'.encode('utf-8'))
                # Define output for visual diagnosis (green LED)
                GPIO.output(led_green, GPIO.LOW)  
                # Read response from PTU-Control after pausing in order to include PTU-response
                # (Reading of complete buffer)
                time.sleep(0.2)
                print('Feedback PTU-Control:', str(ser.read(ser.inWaiting())))
            # Input is False and last cycle was TRUE --> Switch OFF PTU            
            else:
                print("PTU Off")
                ser.write(b'D5_Off\n')
                # Define RPM-Setpoint for stopping (Reset to 0)
                ser.write(b'A0=' + str(0.0).encode('utf-8') +'\n'.encode('utf-8'))
                # Define output for visual diagnosis (green LED)
                GPIO.output(led_green, GPIO.HIGH)        
                # Read response from PTU-Control after pausing in order to include PTU-response
                # (Reading of complete buffer)
                time.sleep(0.2)
                print('Feedback PTU-Control:', str(ser.read(ser.inWaiting())))        
                time.sleep(0.1)
                ser.close()             # Changed 2022-11-30


        # Optional evaluation of additional digital signal in order to define a ramp for the RPM-setpoint
        # Only active, when PTU is switched ON
        # The ramp is realized with a counter and changed for 1/ within the counter intervalls
        # Indicate input for ramping (red LED)
        if ptu_ctrl_ramp_state:
            # Define output for visual diagnosis
            GPIO.output(led_red, GPIO.LOW)  
        else:
            GPIO.output(led_red, GPIO.HIGH) 

        # Differentiate between ramp values:
        # The main influence for the ramp steepness is ths sleep time.
        #Therefore, only that value is assigned depending on the input values
        # Default  value is 0s (meaning no ramp)
        # Steepness is mainly defined by sleep_ramp and rpm increment value
        if GPIO.input(ptu_ctrl_r2) and not GPIO.input(ptu_ctrl_r3):
            sleep_ramp = 0.1          # Steepness ~1%/0,1s 
        elif not GPIO.input(ptu_ctrl_r2) and GPIO.input(ptu_ctrl_r3):
            sleep_ramp = 0.2          # Steepness ~1%/0,2s 
        elif GPIO.input(ptu_ctrl_r2) and GPIO.input(ptu_ctrl_r3):
            sleep_ramp = 0.4          # Steepness ~1%/0,4s 
        else: 
            sleep_ramp = 0            # No ramping, immediate jump to max

	# Reset Counter 
        if not ptu_ctrl_state:
            # Reset counter when no ramp input active
            spm_rpm = sp_rpm_ramp_min
        # The counter value defines the steepness of the ramp
        if ptu_ctrl_ramp_state: 
            # Ramping setpoint
            # Conditions: PTU switched ON, Input INCREASE pressed and ramp setpoint within range
            if ptu_ctrl_state and ptu_ctrl_ramp_state and sp_rpm_ramp_min <= sp_rpm < sp_rpm_ramp_max:
                # After tests on 2022-11-14, direct jump instead of ramping should be the default option   
                if sleep_ramp == 0:
                    sp_rpm = sp_rpm_ramp_max 
                    # With sleep_ramp 0, immediate jump to max output
                    ser.write(b'A0=' + str(sp_rpm).encode('utf-8') +'\n'.encode('utf-8'))  
                    # Sleep time (fixed)
                    time.sleep(0.1)
                    print('No ramping up, direct jump. Selected RPM-setpoint is:', sp_rpm)                   
                else:
                    # Change setpoint and write to controller                            
                    sp_rpm += 1
                    # Write new, actualized RPM-setpoint to controller
                    ser.write(b'A0=' + str(sp_rpm).encode('utf-8') +'\n'.encode('utf-8'))
                    # Diagnosis
                    time_list.append(time.time()/1000) #int(round(time.time() * 1000))
                    rpm_list.append(sp_rpm)
                    # Sleep time assigned as variable in order to parametrize process
                    time.sleep(sleep_ramp)
                    print('Selected RPM-setpoint ramping up is:', sp_rpm) 
            # Print list for evaluation, when ramping up is finished
            if sp_rpm >= sp_rpm_ramp_max:
                if not print_once:
                    print('Sleep value:', sleep_ramp)
                    print('TimeValues:', time_list)
                    print('RPM-Values', rpm_list)
                    print_once = True

        # Switching to minimal value when switching of ramp input
        # Conditions: PTU switched ON and ramp input switched OFF
        if ptu_ctrl_state and not ptu_ctrl_ramp_state and ptu_ctrl_ramp_state_last:
            # Write new, actualized RPM-setpoint to controller
            sp_rpm = sp_rpm_ramp_min
            ser.write(b'A0=' + str(sp_rpm).encode('utf-8') +'\n'.encode('utf-8'))
            time.sleep(0.1)
            print('Setting of minimal RPM-setpoint to:', sp_rpm) 
            

        # Assign value for evaluation
        ptu_ctrl_robot_last = ptu_ctrl_state
        ptu_ctrl_ramp_state_last = ptu_ctrl_ramp_state
# When you press ctrl+c, this will be called
finally:
    GPIO.cleanup()



