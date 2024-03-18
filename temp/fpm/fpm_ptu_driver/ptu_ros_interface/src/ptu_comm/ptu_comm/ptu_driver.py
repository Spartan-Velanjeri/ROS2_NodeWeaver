from serial_interface import SerialInterface


class PTU_Driver():
    
    def __init__(self) -> None:
        
        self.serial_interface = SerialInterface(device_port='/dev/ptu')

    def power_on(self):
        self.serial_interface.write(data=b'D5_On\r\n')

    def power_off(self):
        self.serial_interface.write(data=b'D5_Off\r\n')


if __name__=="__main__":

    PTU = PTU_Driver()

    print("Powering on")
    result = PTU.power_on()

    print(f"Result={result}")

    import time
    time.sleep(5)

    print("Powering off")
    result = PTU.power_off()

    print(f"Result={result}")