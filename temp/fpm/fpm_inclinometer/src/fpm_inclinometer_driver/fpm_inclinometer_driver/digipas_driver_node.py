# import for ROS2
import binascii
import traceback
from typing import Dict

import rclpy

#import for python script
import serial
import serial.tools.list_ports

#from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy import Parameter
from rclpy.node import Node
from scipy.spatial.transform import Rotation

NODE_NAME = "digipas_driver_node"
SENSOR_PERIOD = 0.1 
THROTTLE_VALUE_LOGGING = 5


def angleDualAxis(hexString):
    """
    Movint to digipas_driver class doesn't workin g(causing exception)
    
    Implement test of mode or integrate other modes 
    For DWL5000XY Dual Axis:
        Decimal Degree Y = (((Byte [4] << 16) + (Byte [3]<< 8) + Byte [2]) - 300000) / 10000
        Decimal Degree X = (((Byte [7] << 16) + (Byte [6] << 8) + Byte [5]) - 300000) / 10000
    #!!!!!!!!!!!!!!!!!!
    """
    # # Equation for: Single Axis Mode, Axis 1 
    # In case of wrong splitting (value '71' occurs also within values), string length
    # and start string is checked
    # For diagnosis purposes default definition of values
    angleValue_X = 'NaN'
    angleValue_Y = 'NaN'
    #print('Testing on FPM-Spectra. HexCode: length and start', len(hexString), hexString[0:2]) # Debugging 13.10.2022
#    if len(hexString) == 24 and hexString[0:2] == '61':
    if True:
        # Calculated value refers to Decimal Degree:
        # for performance reasons, there is no additional separation of received hex-code
        # e.g. into a list or blank removal. There's a direct access to the relevant items
        # implemented. Blanks shouldn't exist here.
        # Byte7: char.position 14..16; Byte6: char.position 12..14;
        # Byte5: char.position 10..12; Byte4: char.position 8..10;
        # Byte3: char.position 6..8; Byte2: char.position 4..6

        angleValue_X = ((int(hexString[8:10], base = 16) * (2**16) + int(hexString[6:8], base = 16) * (2**8) 
                    + int(hexString[4:6], base = 16) - 300000) / 10000) 
        
        angleValue_Y = ((int(hexString[14:16], base = 16) * (2**16) + int(hexString[12:14], base = 16) * (2**8) 
                    + int(hexString[10:12], base = 16) - 300000) / 10000) 
        return angleValue_X, angleValue_Y
    else:
        return angleValue_X, angleValue_Y

class digipas_driver(Node):

    def __init__(self):
        # initialisation of node and publisher
        super().__init__(NODE_NAME)

        # declaration of node parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("port", Parameter.Type.STRING),
                ("baudrate", Parameter.Type.INTEGER),
                ("parity", Parameter.Type.STRING),
                ("stopbits", Parameter.Type.DOUBLE),
                ("bytesize",Parameter.Type.INTEGER),
                ("xonxoff", Parameter.Type.BOOL)
            ]
        )

        self.get_logger().info(f"Node parameters: {self.parameters}")

        self.digipas_inclinometer_publisher = self.create_publisher(PoseStamped, 'digipas_inclinometer_data', 10)
        
        # initialisation of timer
        timer_period = SENSOR_PERIOD  # seconds
        self.timer = self.create_timer(timer_period, self.digipas_communication_callback)
        self.i = 0

        # initialisation of RS485 communication
        device_rs485 = '/dev/ttyUSB0'
        # configure the serial connections (the parameters differs on the device you 
        # are connected to, e.g. USB-COM may change depending on connected items)
        self.ser = serial.Serial(
            port=self.get_parameter("port").value,
            baudrate=self.get_parameter("baudrate").value,
            parity=self.get_parameter("parity").value,
            stopbits=self.get_parameter("stopbits").value,
            bytesize=self.get_parameter("bytesize").value,
            xonxoff= self.get_parameter("xonxoff").value,
        )
        # For unclear reasons, the port is already in use from an application or Python
        # and can't be found then.
        # Therefore closing here via TRY/EXCEPT
        try:
            self.ser.close()
        except:
            self.get_logger().warning('Port could not be closed, probably already closed.')  

        self.ser.open()
        self.get_logger().debug(f"main Opened?: {self.ser.is_open}")
        
        # Init data
        """
        Buffer Format : Hexadecimal
        Buffer Length : 12 bytes

        SEND:
        The data to be transmitted out to tilt sensor module is as follow:
        Source Destination Data
        1 Byte 1 Byte 10 Byte

        RECEIVE:
        The data to be received from the tilt sensor module is as follow:
        Source/Destination Stand/Mode Data CRC16
        1 Byte 1 Byte 8 Bytes 2 Bytes

        """
        # Init-mode for directly connected sensor
        command = b'\x06\x24\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.ser.write(command)
        self.get_logger().info(f"InitCommand written:{str(command)}")
        # The init code activates the sensor to set it to standard dual axis mode
        # and starts sending signals

    @property
    def parameters(self) -> Dict:
        """Returns parameters.

        Returns:
            Dict: Parameters.
        """
        parameters = {}

        for parameter in self.get_parameters_by_prefix(prefix="").keys():
            parameters.update(
                {parameter: self.get_parameter(name=parameter).value}
            )

        return parameters
    
    def digipas_communication_callback(self):
        bufLength = self.ser.inWaiting()
        #print('BufferLength', bufLength)           
        #print('ser.read:', self.ser.read(12))
        #print('hexString:', str(binascii.hexlify(self.ser.read(12)))[2:-1])

        try:
            angleValue_X, angleValue_Y = angleDualAxis(str(binascii.hexlify(self.ser.read(12)))[2:-1]) 
            self.get_logger().info(f"Angles X/Y:{angleValue_X}/{angleValue_Y}",throttle_duration_sec=THROTTLE_VALUE_LOGGING)    
        except:
            # Default values when no data could be retrieved
            angleValue_X = 99.99
            angleValue_Y = 99.99
            self.get_logger().error('No data could be read from serial interface/Delock RS485-Converter')
            pass

        #Reset buffer after reading in order to receive actual data
        self.ser.flushInput()       

        # convert data to PoseStamp, adaptation 2022-10-18
        inclinometer_rotation = Rotation.from_euler('zyx', [0.0, angleValue_Y, angleValue_X], degrees=True)
        inclinometer_quat = inclinometer_rotation.as_quat()
        inclinometer_data_msg = PoseStamped()
        inclinometer_data_msg.header.stamp = self.get_clock().now().to_msg()
        inclinometer_data_msg.header.frame_id = "digipas_inclinometer_base"
        inclinometer_data_msg.pose.orientation.x = inclinometer_quat[0]
        inclinometer_data_msg.pose.orientation.y = inclinometer_quat[1]
        inclinometer_data_msg.pose.orientation.z = inclinometer_quat[2]
        inclinometer_data_msg.pose.orientation.w = inclinometer_quat[3]


        self.digipas_inclinometer_publisher.publish(inclinometer_data_msg)


def main(args=None):
    print('Hi from fpm_inclinometer_driver. DigiPas DWL5500')
    rclpy.init(args=args)

    try:
        inclination_publisher_node = digipas_driver()

        inclination_publisher_node.get_logger().info(f"Starting {NODE_NAME} node.")

        try:
            rclpy.spin(inclination_publisher_node)
        
        except KeyboardInterrupt:
            inclination_publisher_node.get_logger().fatal("Inclination_publisher_node keyboard interrupt.")

        except BaseException as e:
            eString = "".join(
                traceback.format_exception(
                    etype=type(e), value=e, tb=e.__traceback__
                )
            )
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"{NODE_NAME}: Unknown exception:{eString}."
            )
        
        inclination_publisher_node.get_logger().fatal("Inclination_publisher_node driver shuting down.")
        inclination_publisher_node.destroy_node()
        rclpy.shutdown()

    except serial.SerialException as e:

        eString = "".join(
            traceback.format_exception(
                etype=type(e), value=e, tb=e.__traceback__
            )
        )

        if e.errno == 13:
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"USB permission denied: {e.strerror}."
            )
        if e.errno == 2:
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"Device not connected: {e.strerror}."
            )            
        else:
            Node(node_name=NODE_NAME).get_logger().fatal(
            f"Unknow exception: {eString}."
        )

    except BaseException as e:
        eString = "".join(
            traceback.format_exception(
                etype=type(e), value=e, tb=e.__traceback__
            )
        )
        Node(node_name=NODE_NAME).get_logger().fatal(
            f"Unknow exception: {eString}."
        )

if __name__ == '__main__':
    main()
