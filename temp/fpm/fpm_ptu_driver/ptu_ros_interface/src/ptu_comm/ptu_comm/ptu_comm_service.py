"""
# -*- coding: utf-8 -*-
# Copyright (C) Robert Bosch GmbH 2022.
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
"""

"""
Basic funcionality based on example from:
https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

Module description
------------------
Defining signals for controlling the PTU-interface. The interface is realized on a 
MicroController (Type: AdaFruit Feather M4 Express) with CANopen-Interface and serial
communication. 
There are three individual and different control options as a minimal interface usedfor the PTU:
- Switch ON
- Turn RIGHT/LEFT
- Define analog setpoint
In a first and simple approach the parameters when calling the client are realized with integer values
with the following meaning:
    1:          Switch PTU On
    2:          Switch PTU Off
    11:         Turn PTU Left
    12:         Turn PTU Right
    100..200:   Setpoint in % for 0..100% (subtraction of 100 to parameter value)

The strings sent to the µC (e.g. D5_On...) refer to the used digital or analog pin on the electronic board. 

If successfull the Microcontroller responds with a string containing the substring 'PTU' followed by an 
addition substring to describe the performed action.
This string is taken as response to the ROS-system.

The service description with respct to input/output parameters is centrally defined for the bautiro-project
in the bautiro_ros_interfaces folder. 
The description used here is FpmPtuComm.srv
"""

"""
__author__ = "Martin Dieterle (die2si)"
__license__ = "Bosch Internal Open Source License v4"
__email__ = "martin.dieterle@de.bosch.com"
__status__ = "Prototype" # Can be Prototype, Development, Production
"""

# Added after cloning bautiro_ros_interfaces
from bautiro_ros_interfaces.srv import FpmPtuComm    # Added 2022-10-17
from symbol import return_stmt

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import time


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FpmPtuComm, 'ptu_communication', self.ptu_communication_callback)

    def ptu_communication_callback(self, request, response):
        
        # For a LINUX_System
        ser = serial.Serial(
            port= '/dev/ttyACM0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            xonxoff= False  
        )

        # Ensure an opened COM-port:
        if not ser.is_open:
            ser.open()
            print('main Opened?:', ser.is_open)
            print('Connection name', ser.name)         # check which port was really used
        """    
        try: 
            ser.open()
            print('main Opened?:', ser.is_open)
            print('Connection name', ser.name)         # check which port was really used
        except:
            print('Serial open with error, probably already opened')
            pass
        """
        # Defining serial strings based on client parameter:
        if request.contr_int == 1:
            # String for writing to COM-Interface
            ser.write(b'D5_On\r\n')
            print('Write to µC: D5_On')
        elif request.contr_int == 2:
            # String for writing to COM-Interface
            ser.write(b'D5_Off\r\n')  
            print('Write to µC: D5_Off')  
        elif request.contr_int == 11:
            # String for writing to COM-Interface
            ser.write(b'D6_R\r\n')
            print('Write to µC: D6_R')
        elif request.contr_int == 12:
            # String for writing to COM-Interface
            ser.write(b'D6_L\r\n')  
            print('Write to µC: D6_L') 
        elif 100 <= request.contr_int  <= 200:
            # Analog setpoint:
            # String for writing to COM-Interface, concat and encode to byte array
            ser.write(str.encode('A0=')+str.encode(str(request.contr_int - 100))+str.encode('\r\n'))  
            print('Write to µC: A0=')              
        else:
            pass  
        # 0,2s required, with just 0.1, only comannd line readable
        time.sleep(0.2)
        # Read complete buffer and evaluate it
        # Number of bytes in the input buffer
        bufferBytes = ser.inWaiting()
        returnString = str(ser.read(bufferBytes))
        # Extract substring starting from 'PTU' to first CRLF in order to identify the plain response of µC 
        # Approach with splitlines doesn't work within ROS ('str' object has no attribute splitlines)
        µC_Response  = returnString[returnString.find('PTU') :][: returnString[returnString.find('PTU'):].find('\\r')]
        print('Response of µC: ', µC_Response)
        # Close COM-Interface
        ser.close()
        response.ptu_response = µC_Response # str(request.contr_int) 
        self.get_logger().info('Incoming request for contr_int: %d' % (request.contr_int))

        return response


def main():
    rclpy.init()
    # Instantiate service
    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()