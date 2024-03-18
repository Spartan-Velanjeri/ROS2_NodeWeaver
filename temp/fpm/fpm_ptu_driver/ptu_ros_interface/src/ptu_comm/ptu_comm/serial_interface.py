import serial
import serial.tools.list_ports


class SerialInterface():

    def __init__(self, device_port:str) -> None:
        
        self.device = serial.Serial(
            port= device_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            xonxoff= False  
        )

    @property 
    def port_open(self) -> bool:
        return self.device.is_open

    def open_port(self):
        if not self.port_open:
            self.device.open()  
  
    def write(self,data:bytes):
        return self.device.write(data)

if __name__ == "__main__":

    port = '/dev/ttyACM0'

    driver = SerialInterface(device_port= port)

