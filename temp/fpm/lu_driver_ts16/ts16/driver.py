#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

"""Driver for Leica TS16."""

import signal
import sys
import threading
import time
from os import path
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import TimeReference

from ts16.playback_interface import PlaybackInterface
from ts16.serial_interface import SerialInterface
from bautiro_ros_interfaces.srv import LuTsMeasurePoint, LuTsMeasurePoints
from ts16.process_ros_parameters import process_ros_parameters

class LeicaDriver(Node):
    """Driver for Leica TS16."""

    # Launch arguments
    launch_args={
        'serial_port': {
            'description': 'serial port access ts16',
            'default': '/dev/ttyUSB1',
        },
        'serial_baud': {
            'description': 'baud rate of serial connection to ts16',
            'default': '115200',
        },
        'interface': {
            'description': 'ts16 interface selection: "wlan", "usb", "serial", "playback"',
            'default': 'serial',
        },
        'logging': {
            'description': 'activate or deactivate logging into file: "True" or "False"',
            'default': 'False',
        },
        'mode': {
            'description': 'FPM_LOCALIZATION mode: simulation or real hardware (gazebo_sim, real)',
            'default': 'real',
        },                              
    }    
    
    def __init__(self):
        super().__init__('ts16')

        #create service for control ts16
        self.srv = self.create_service(LuTsMeasurePoint, 'lu_control_ts16', self.service_callback_xyz_one)
        self.srv_multi = self.create_service(LuTsMeasurePoints,'lu_control_ts16_multi', self.service_callback_xyz_multi)

        # TS16 clock drift in s pro h
        self.declare_parameter('clock_drift', 0.139)
        #self.declare_parameter('debug_mode', False)
        #self.declare_parameter('tracking_mode', False)

        self.launch_args = process_ros_parameters(self, self.launch_args)
        self.interface_selection = self.launch_args['interface']['value']
        self.logging = eval(self.launch_args['logging']['value'])
        self.mode = self.launch_args['mode']['value']
        self.clock_drift         = self.get_parameter('clock_drift').value
        self.logging_path = path.join('/tmp', 'leica_log_' + '_' + str(time.strftime('%Y-%m-%d_%H-%M-%S')) + '.log')
        #self.debug_mode = self.get_parameter('debug_mode').value
        self.input_format = 'Pt_E_N_Ht_Date'
        readme_filename = get_package_share_directory('ts16') + '/config/readme.yaml'
        with open(readme_filename, 'r') as s:
            readme = yaml.safe_load(s)

        commands_filename = get_package_share_directory('ts16') + '/config/commands.yaml'
        with open(commands_filename, 'r') as s:
            self.commands = yaml.safe_load(s)

        answers_filename = get_package_share_directory('ts16') + '/config/answers.yaml'
        with open(answers_filename, 'r') as s:
            self.answers = yaml.safe_load(s)

        message_fields_filename = get_package_share_directory('ts16') + '/config/message_fields.yaml'
        with open(message_fields_filename, 'r') as s:
            self.message_fields = yaml.safe_load(s)

        #self.tracking_mode = self.get_parameter('tracking_mode').value

        self.command_names = {v: k for k, v in self.commands.items()}

        if self.input_format != 'Pt_E_N_Ht_Date':
            self.get_logger().warn('Currently only the input format Pt_E_N_Ht_Date is implemented. Switching to this format.')
            self.input_format = 'Pt_E_N_Ht_Date'

        self.translator = self.message_fields[self.input_format]
        self.time_offset = None
        self.time_initialization = None
        self.busy = False

        if self.logging:
            try:
                self.log = open(self.logging_path, 'a')
            except IOError:
                self.get_logger().warn("Failed to access logging path.")
                self.logging = False

        if self.mode == 'real':
            self.get_logger().info('\n-----\n\n' + readme[self.interface_selection] + '\n' + readme['suffix'])   
            if self.interface_selection == 'serial':
                self.declare_parameter('serial_timeout',10)
                serial_port = self.launch_args['serial_port']['value']
                serial_baud = int(self.launch_args['serial_baud']['value'])
                self.serial_timeout = self.get_parameter('serial_timeout').value
                self.get_logger().info('Setting Serial Communication for %s @ %i timeout=%i ...'%(serial_port,serial_baud,self.serial_timeout))
                self.interface = SerialInterface(serial_port=serial_port,serial_baud=serial_baud,serial_timeout=self.serial_timeout)
            elif self.interface_selection in ['wlan', 'usb']:
                self.get_logger().error('Interface {} is not supported by TS16!'.format(self.interface_selection))
                sys.exit(0)
            elif self.interface_selection == 'playback':
                self.declare_parameter('playback_path','/tmp/tty')
                playback_path = self.get_parameter('playback_path').value            
                self.interface = PlaybackInterface(playback_path)
            else:
                self.get_logger().error('Unsupported interface {} selected'.format(self.interface_selection))
                sys.exit(0)

            self.interface.connect()

        elif self.mode == "gazebo_sim":
            self.get_logger().info('Currently Used Mode: ' + self.mode)

        else:
            self.get_logger().error("No valid mode selected (real, gazebo_sim), selected mode is: " + self.mode)
            exit()

        # send initialization commands
        if self.mode == "real":  # interface not available for self.mode == "gazebo_sim"
            if not self.interface_selection == 'playback':
                self.initialize_system()
            else:
                self.time_initialization = time.time()
                self.time_offset = self.interface.time_sync(self.time_initialization)

        self.pub_position = self.create_publisher(PointStamped, 'position', 10)
        # self.pub_position_no_drift_conpensation = self.create_publisher(PointStamped, 'position_no_drift_compensation', 10)
        # self.pub_position_raw = self.create_publisher(PointStamped, 'position_raw_timing', 10)
        # self.pub_timeref = self.create_publisher(TimeReference, 'timeref', 10)

        self.sub_measure_at_position = self.create_subscription(PointStamped, 'target_position_coord', self.callback_xyz_one, 10)
        self.sub_measure_at_position_hv = self.create_subscription(PointStamped, 'target_position_hz_v', self.callback_hv_one, 10)

        # self.th = threading.Thread(target=self.read, args=())
        # self.th.daemon = True
        # self.th.start()

        self.get_logger().info('Leica TS16 ready and waiting for commands ...')


    def initialize_system(self):
        """Initialize the TS16."""

        # stop distance
        self.get_logger().info("Stop the distance procedure if it is running ...")
        ans = self.command_send_receive(self.commands['stop_distance'],'stop_distance')
        if ans[0:-2]==self.answers['GRC_OK'][0]:
            self.get_logger().info(self.answers['GRC_OK'][1])
        elif ans[0:-2]==self.answers['GRC_STOP_DIST_FAILED'][0]:
            self.get_logger().error(self.answers['GRC_STOP_DIST_FAILED'][1])
            self.destroy_node()
            rclpy.shutdown()
        elif ans=='timeout':
            self.destroy_node()
            rclpy.shutdown()
            rclpy.shutdown()
        else:
            self.get_logger().error("Unknown answer.")
            self.destroy_node()
            rclpy.shutdown()

        # stop streaming
        self.get_logger().info("Stop the streaming from the configured interface ...")
        ans = self.command_send_receive(self.commands['stop_stream'],'stop_stream')
        if ans[0:-2]==self.answers['GRC_OK'][0]:
            self.get_logger().info(self.answers['GRC_OK'][1])
        elif ans[0:-2]==self.answers['GRC_STREAM__NOT_ACITVE'][0]:
            self.get_logger().info(self.answers['GRC_STREAM__NOT_ACITVE'][1])
        elif ans[0:-2]==self.answers['GRC_STOP_FAILED'][0]:
            self.get_logger().error(self.answers['GRC_STOP_FAILED'][1])
            self.destroy_node()
            rclpy.shutdown()
        elif ans=='timeout':
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().error("Unknown answer.")
            self.destroy_node()
            rclpy.shutdown()


    def command_send_receive(self, command, command_str='???'):
        """Send a command and return the answer

        Args:
            command: command which shall be send, e.g. 'start_stream'
        """
        # if not self.debug_mode:
        #     try:
        #         self.get_logger().info('Sending command {}'.format(self.command_names[command[:7]]))
        #     except KeyError:
        #         self.get_logger().info('Sending command {}'.format(self.command_names[command]))
        # else:
        #     self.get_logger().info('Sending command {}'.format(command))

        self.get_logger().info('Sending command {} ({})'.format(command,command_str))

        self.interface.write('\n'.encode() + command.encode() + '\r\n'.encode())

        t_0 = time.time()
        ans = self.interface.readlines(single=True)
        t_1 = time.time()

        if ans=='' and (t_1-t_0)>self.serial_timeout:
            self.get_logger().warn("Timeout in communication (%s)!"%(command_str))
            ans = 'timeout'

        return ans


    def measure_one_point(self,x,y,z):
        '''Measure one point'''

        if self.mode == "real":
            x_out = 0.0
            y_out = 0.0
            z_out = 0.0
    
            # Move the telescope to the target position #########
            self.get_logger().info("Turning the telescope ...")
            ans = self.command_send_receive(self.commands['turn_telescope'] + "1," + "%f,%f,%f"%(x,-y,z),'turn_telescope')

            # Problem with USB Hub / long USB cable on FUS1
            # Close and reopen the communication interface and try again
            if ans == 'timeout':
                self.get_logger().warn('Timeout in communication, closing and reopening the communication interface!')
                self.interface.close()
                time.sleep(1.0)
                self.interface.connect()
                ans = self.command_send_receive(self.commands['turn_telescope'] + "1," + "%f,%f,%f"%(x,-y,z),'turn_telescope')

            if ans[0:-2]==self.answers['GRC_NOT_OK'][0]:
                self.get_logger().warn(self.answers['GRC_NOT_OK'][1])
            elif ans[0:-2]==self.answers['GRC_PositioningFailed'][0]:
                self.get_logger().error(self.answers['GRC_PositioningFailed'][1])
                self.destroy_node()
                rclpy.shutdown()
            elif ans[0:-2]==self.answers['GRC_OK'][0]:
                self.get_logger().info(self.answers['GRC_OK'][1])

                # Start searching ###############################
                self.get_logger().info("Start searching ...")
                ans = self.command_send_receive(self.commands['start_distance'],'start_distance')
                if ans[0:-2]==self.answers['GRC_DIST_ERR_NOT_DOCUMENTED'][0]:
                    self.get_logger().warn(self.answers['GRC_DIST_ERR_NOT_DOCUMENTED'][1])
                else:
                    self.get_logger().info("Marker/Prisma found.")

                    # Start streaming ##########################
                    self.get_logger().info("Start streaming ...")
                    ans = self.command_send_receive(self.commands['start_stream'],'start_stream')
                    if ans[0:-2]==self.answers['GRC_STREAM_ACITVE'][0]:
                        self.get_logger().info(self.answers['GRC_STREAM_ACITVE'][1])
                    elif ans[0:-2]==self.answers['GRC_START_FAILED'][0]:
                        self.get_logger().error(self.answers['GRC_START_FAILED'][1])
                        self.destroy_node()
                        rclpy.shutdown()
                    elif ans[0:-2]==self.answers['GRC_OK'][0]:
                        self.get_logger().info(self.answers['GRC_OK'][1])

                        # Read the distance ####################
                        line = self.interface.readlines(single=True)
                        if line==None:
                            self.get_logger().warn("The distance could not be read! Timeout reached! Trying again ...")
                            line = self.interface.readlines(single=True)
                            if line==None:
                                self.get_logger().warn("The distance could not be read! Timeout reached!")
                        else:
                            ans = self.command_send_receive(self.commands['stop_stream'],'stop_stream')
                            ans = self.command_send_receive(self.commands['stop_distance'],'stop_distance')

                            x_out,y_out,z_out = self.process_data(line)

                            if self.logging:
                                self.log.write(line)
            elif ans == 'timeout':
                self.get_logger().error("Timeout in communication with TS16 reached: Please check whether the Measure&Stream app is running!")

        else:   #self.mode == "gazebo_sim"
            x_out = x
            y_out = y
            z_out = z

        return x_out,y_out,z_out


    def check_validity(self,x,y,z):
        '''Check where or not the target position is valid'''

        class valid_range:
            x_min = -1e2 # initial value, to be validated!
            y_min = -1e2 # initial value, to be validated!
            z_min = -1e2 # initial value, to be validated!
            x_max = 1e2 # initial value, to be validated!
            y_max = 1e2 # initial value, to be validated!
            z_max = 1e2 # initial value, to be validated!

        if not valid_range.x_min < x < valid_range.x_max:
            self.get_logger().warn("X coordinate %f out of range! Valid range [%.1f .. %.1f]"%(x,valid_range.x_min,valid_range.x_max))
            return False
        elif not valid_range.y_min < y < valid_range.y_max:
            self.get_logger().warn("Y coordinate %f out of range! Valid range [%.1f .. %.1f]"%(y,valid_range.y_min,valid_range.y_max))
            return False
        elif not valid_range.z_min < z < valid_range.z_max:
            self.get_logger().warn("Z coordinate %f out of range! Valid range [%.1f .. %.1f]"%(z,valid_range.z_min,valid_range.z_max))
            return False

        return True


    def process_data(self, data):
        """Process the data received from the total station.

        Args:
            data: ASCII line that shall be processed
        """

        # ignore ACK messages
        if data.strip() == '%R8P,0,0:0':
            return

        data_raw = data.strip().split(',')
        try:
            data = dict(zip(self.translator, data_raw))
            x = float(data['Northing'])
            y = -float(data['Easting'])
            z = float(data['Elevation'])
        except (KeyError, ValueError, TypeError) as e:
            self.get_logger().warn('Incomplete message received: {}, {}'.format(e, data_raw))
        
        return x,y,z
    

    def callback_xyz_one(self,msg):
        '''Callback for the topic with a single point to measure'''

        if self.busy == False:

            self.busy = True

            x_tar = msg.point.x
            y_tar = msg.point.y
            z_tar = msg.point.z

            # Check where or not the target position is valid
            if not self.check_validity(x_tar,y_tar,z_tar):
                self.busy = False
                return
            
            x,y,z = self.measure_one_point(x_tar,y_tar,z_tar)
            
            if x==0.0 and y==0.0 and z==0.0: # not valid measurement
                self.get_logger().warn('Nothing published.')
            else:
                time_now = self.get_clock().now()
                msg_position = PointStamped()
                msg_position.header.frame_id      = msg.header.frame_id 
                msg_position.header.stamp.sec     = int(time_now.nanoseconds/1e9)
                msg_position.header.stamp.nanosec = int(time_now.nanoseconds - msg_position.header.stamp.sec*1e9)
                msg_position.point.x = x
                msg_position.point.y = y
                msg_position.point.z = z
                self.pub_position.publish(msg_position)
                self.get_logger().info('Published MEASUREMENT: (%.3f, %.3f, %.3f)'%(msg_position.point.x,msg_position.point.y,msg_position.point.z))
                print('')

            self.busy = False

        else:
            self.get_logger().warn("Device is busy! Ignoring this message!")



    def callback_hv_one(self,msg):
        self.get_logger().error("Callback not implemented yet!")
        return


    def service_callback_xyz_one(self, request, response):
        '''Callback for the service with a single point to measure'''

        # get input points from request
        msg = request.in_point

        if self.busy == False:

            self.busy = True

            x_tar = msg.point.x
            y_tar = msg.point.y
            z_tar = msg.point.z

            # Check where or not the target position is valid
            if not self.check_validity(x_tar,y_tar,z_tar):
                self.busy = False
                return
        
            x,y,z = self.measure_one_point(x_tar,y_tar,z_tar)
            
            if x==0.0 and y==0.0 and z==0.0: # not valid measurement
                self.get_logger().warn('Returned MEASUREMENT: (0,0,0)!')
                print('')
            else:
                time_now = self.get_clock().now()
                msg_position = PointStamped()
                msg_position.header.frame_id      = msg.header.frame_id 
                msg_position.header.stamp.sec     = int(time_now.nanoseconds/1e9)
                msg_position.header.stamp.nanosec = int(time_now.nanoseconds - msg_position.header.stamp.sec*1e9)
                msg_position.point.x = x
                msg_position.point.y = y
                msg_position.point.z = z
                response.out_point = msg_position
                self.get_logger().info('Returned MEASUREMENT: (%.3f, %.3f, %.3f)'%(msg_position.point.x,msg_position.point.y,msg_position.point.z))
                print('')

            self.busy = False
            
            return response

        else:
            self.get_logger().warn("Device is busy! Ignoring this message!")


    def service_callback_xyz_multi(self, request, response):
        '''Callback for the service with a list of points to measure'''

        # get input points from request
        msgs = request.in_point
        self.get_logger().info('Service request with %i points ...'%(len(msgs)))
        print('')

        if self.busy == False:

            self.busy = True
            response.out_point = []

            for msg in msgs:

                x_tar = msg.point.x 
                y_tar = msg.point.y
                z_tar = msg.point.z

                # Check where or not the target position is valid
                # and measure
                if self.check_validity(x_tar,y_tar,z_tar):
                    x,y,z = self.measure_one_point(x_tar,y_tar,z_tar)
                else:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    self.get_logger().warn('The target position (%.3f, %.3f, %.3f) is not valid!'%(x_tar,y_tar,z_tar))
                    
                # Create message header
                time_now = self.get_clock().now()
                msg_position = PointStamped()
                msg_position.header.frame_id      = msg.header.frame_id 
                msg_position.header.stamp.sec     = int(time_now.nanoseconds/1e9)
                msg_position.header.stamp.nanosec = int(time_now.nanoseconds - msg_position.header.stamp.sec*1e9)

                if x==0.0 and y==0.0 and z==0.0: # not valid measurement
                    self.get_logger().warn('Returned MEASUREMENT: (0,0,0)!')
                    print('')
                else: # valid message
                    msg_position.point.x = x
                    msg_position.point.y = y
                    msg_position.point.z = z
                    self.get_logger().info('Returned MEASUREMENT: (%.3f, %.3f, %.3f)'%(msg_position.point.x,msg_position.point.y,msg_position.point.z))
                    print('')

                response.out_point.append(msg_position)

            self.busy = False
            
            self.get_logger().info('Respond to the service with %i measurements.'%(len(response.out_point)))
            return response

        else:
            self.get_logger().warn("Device is busy! Ignoring this request!")


# def termination_handler(*_):
#     """Handle keyboard Ctrl+C interrupts."""
#     self.get_logger().warn('User interrupt, exiting...')
#     node.send_command(node.commands['stop_stream'])
#     time.sleep(0.5)
#     node.send_command(node.commands['stop_distance'])
#     time.sleep(0.5)
#     node.interface.close()
#     sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    node = LeicaDriver()
    # node.th.join()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()

