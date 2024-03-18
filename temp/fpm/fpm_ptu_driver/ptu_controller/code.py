"""CircuitPython
Digital Out and Analog Out management.
The serial input strings are transfered to digital
resp. analog output values.
The following strings are considered to be valid:
Bohrhammer ON/OFF:
 D5_On --> Bohrhammer ON
 D5_Off --> Bohrhammer OFF
 D6_L --> Bohrhammer turning direction LEFT
 D6_R --> Bohrhammer turning direction RIGHT
 A0=xxx --> Bohrhammer setpoint rpm (expected value 0..100%)

The input data are concatenated until return is hit. This raw data string is
evaluated for its value and an action resp. return value derived from it.

ToDo:
CAN-value:   --> 18.7.2022
Init-String   --> 18.7.2022
Live-Bit  --> 18.7.2022
Restart (STRG-D)
Differenzierte Eingabegrenzwerte
Success-Rückmeldung
"""

import board
import canio

from analogio import AnalogOut
import digitalio
import time
import circuitpython_schedule as schedule
from adafruit_datetime import datetime, timedelta
import re


class USBSerialReader:
    """Read a line from USB Serial (up to end_char), non-blocking, with optional echo"""

    def __init__(self):
        self.s = ""

    def read(self, end_char="\n", echo=True):
        import sys, supervisor

        n = supervisor.runtime.serial_bytes_available
        if n > 0:  # we got bytes!
            s = sys.stdin.read(n)  # actually read it in
            if echo:
                sys.stdout.write(s)  # echo back to human
            self.s = self.s + s  # keep building the string up
            if s.endswith(end_char):  # got our end_char!
                rstr = self.s  # save for return
                self.s = ""  # reset str to beginning
                return rstr
        return None  # no end_char yet


def is_number_tryexcept(s):
    """ Returns True is string is a number.
        In case of a comma separated number/flaot, comma
        is replaced by dot.
    """
    if ',' in s:
        s = s.replace(',','.')
    try:
        float(s)
        return True
    except ValueError:
        return False

# CAN-Bus
# Right above it, there is a small 3V->5V switched-capacitor boost converter so that the output CAN signals are +-5V.
# Use this line if your board has dedicated CAN pins. (Feather M4 CAN and Feather STM32F405)
# Required, otherwise no message is received in time (None is returned). Otherwise, a Message or RemoteTransmissionRequest is returned.
# If the CAN transceiver has a standby pin, bring it out of standby mode
if hasattr(board, 'CAN_STANDBY'):
    standby = digitalio.DigitalInOut(board.CAN_STANDBY)
    standby.switch_to_output(False)
# If the CAN transceiver is powered by a boost converter, turn on its supply
if hasattr(board, 'BOOST_ENABLE'):
    boost_enable = digitalio.DigitalInOut(board.BOOST_ENABLE)
    boost_enable.switch_to_output(True)
can = canio.CAN(rx=board.CAN_RX, tx=board.CAN_TX, baudrate=250_000, auto_restart=True)
#Start receiving messages that match any one of the filters.
#listener = can.listen(matches=[canio.Match(0x181)], timeout=1.9)
#An empty filter list causes all messages to be accepted.
listener = can.listen(matches=[], timeout=.5)

# Define read interval time
CAN_ReadInterval = 0.5
# Time of last read process
CAN_LastRead = -1
old_bus_state = None
old_count = -1

# Assign hardware to variables and define type of variable/output
# DIGITAL SIGNALS
# Hammer ON and OFF (digital output connected to micro relais)
deviceOnOff = digitalio.DigitalInOut(board.D5)
deviceOnOff.direction = digitalio.Direction.OUTPUT
# Hammer direction LEFT and RIGHT (digital output connected to micro relais)
deviceLR = digitalio.DigitalInOut(board.D6)
deviceLR.direction = digitalio.Direction.OUTPUT
# ANALOG SIGNAL
analog_out = AnalogOut(board.A0)
# Code
digOutCounter = 0
switchDigOut = False
cnt = 0

# Define livebit data
livebit = False
# Define read interval time
livebitInterval = 1
# Time of last read process
livebitLastRead = -1

usb_reader = USBSerialReader()
print("Bohrhammer-Interface: Init-State")
print("type something and press the end_char")
while True:
    #-----------------------------------------------
    # Handling digital and analog hardware interface
    # Read raw data string from serial interface
    rawInputStr = usb_reader.read()  # read until newline, echo back chars
    # rawInputStr = usb_reader.read(end_char='\t', echo=False) # trigger on tab, no echo
    if rawInputStr:
        # Clear raw data (remaining are numbers, letters and the characters '_', '.', ',', '=') and replace comma
        # in case it's used as decimal separator
        clearedInputStr = re.sub(r"[^a-zA-Z0-9_=,. ]","", rawInputStr).replace(',','.')
        print('Cleared input string:', clearedInputStr, 'and in Bytes:',list(map(bin,bytearray(clearedInputStr))))

        if clearedInputStr == "D5_On":
            deviceOnOff.value = True
            print("Bohrhammer ON")
        elif clearedInputStr == "D5_Off":
            deviceOnOff.value = False
            print("Bohrhammer OFF")
        elif clearedInputStr == "D6_L":
            deviceLR.value = True
            print("Bohrhammer LEFT")
        elif clearedInputStr == "D6_R":
            deviceLR.value = False
            print("Bohrhammer RIGHT")
        elif clearedInputStr.startswith('A0='):
            analValue = clearedInputStr.split('=')[-1]
            print('Analogwert:', analValue)
            analValueCheck = str(analValue, 'utf-8')
            if is_number_tryexcept(analValueCheck):
                # Check for range, negative values are ignored (since - is removed by clearing code)
                if int(float(analValue)) >= 0.0 and int(float(analValue)) <=100.0:
                    # Expected input range is 0-100%. For full scale adaptation
                    # to DAQ-output (0..65536), multiplication with scaling factor
                    analog_out.value = int(float(analValue) * 655.36)
                    print("Sollwert Bohrhammer", analValue, '%')
                else:
                    print("Sollwert Bohrhammer außerhalb des zugelassenen Bereichs (0..100%)", analValue)
            else:
                print("Ungültiger Wert für Sollwert Bohrhammer", analValue)
        else:
            cnt += 1
            print("Invalid input value:", rawInputStr)

    #-----------------------------------------------
    # Handling CAN-Interface
    # Read CAN-telegram in regular, equidistant intervals
    # Store the current time to refer to later.
    CAN_now = time.monotonic()
    # Check time intervall for reading CAN-telegram:
    if CAN_now >= CAN_LastRead + CAN_ReadInterval:
        CAN_LastRead = CAN_now

        bus_state = can.state
        # state ERROR_ACTIVE indicates, that bus is operational
        if bus_state != old_bus_state:
            print(f"Bus state changed to {bus_state}")
            old_bus_state = bus_state
        print('Bus State:', bus_state)
        message = listener.receive()
        message_in_wait = listener.in_waiting()
        if message is None:
            print("No messsage received within timeout")
            continue
        # Assign message content
        data = message.data
        print('Message data:', data, 'at time', str(CAN_now))
        if len(data) != 8:
            print(f"Unusual message length {len(data)}")
            continue

    #-----------------------------------------------
    # Handling livebit
    # Read CAN-telegram in regular, equidistant intervals
    # Store the current time to refer to later.
    livebitNow = time.monotonic()
    # Check time intervall for reading CAN-telegram:
    if livebitNow >= livebitLastRead + livebitInterval:
        livebitLastRead = livebitNow
        # Define livebit
        if livebit:
            livebit = False
        else:
            livebit = True
        print('Livebit:', livebit)

    time.sleep(0.01)  # do something time critical
