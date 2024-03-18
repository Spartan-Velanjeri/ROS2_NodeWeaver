- [USB Rules for the FPM-CU computer](#usb-rules-for-the-fpm-cu-computer)
- [Rules](#rules)
- [Devices](#devices)
  - [Digipas inclinometer](#digipas-inclinometer)
  - [Le16 leica total station (connected over RS232/USB Adapter)](#le16-leica-total-station-connected-over-rs232usb-adapter)
  - [Power tool unit (PTU)](#power-tool-unit-ptu)
  - [Xsens inclinometer](#xsens-inclinometer)
- [Instalation](#instalation)
- [Literature](#literature)

# USB Rules for the FPM-CU computer

**udev** supplies the system software with device events, manages permissions of device nodes and may create additional symlinks in the /dev/ directory, or renames network interfaces. The kernel usually just assigns unpredictable device names based on the order of discovery. Meaningful symlinks or network device names provide a way to reliably identify devices based on their properties or current configuration.

# Rules
| Rule          	| Device            	| USB symlink 	| Device Info 	|
|-----------------	|--------           	|-------------	|-------------	|
|digipas.rules  	|digipas inclinometer  	|             	|             	|
|le16.rules     	|Leica total station   	|             	|             	|
|ptu.rules      	|Power tool unit       	|             	|             	|
|xsens.rules    	|IMU sxsens         	|             	|             	|

# Devices
* Device information ontained by command *"usb-devices'* in terminal
## Digipas inclinometer
Bus=01 Lev=03 Prnt=41 Port=00 Cnt=01 Dev#= 43 Spd=12  MxCh= 0\
D:  Ver= 2.00 Cls=00(>ifc ) Sub=00 Prot=00 MxPS= 8 #Cfgs=  1\
P:  Vendor=0403 ProdID=6001 Rev=06.00\
S:  Manufacturer=FTDI\
S:  Product=FT232R USB UART\
S:  SerialNumber=A602AMTP\
C:  #Ifs= 1 Cfg#= 1 Atr=a0 MxPwr=90mA
I:  If#=0x0 Alt= 0 #EPs= 2 Cls=ff(vend.) Sub=ff Prot=ff Driver=ftdi_sio
## Le16 leica total station (connected over RS232/USB Adapter)
T:  Bus=01 Lev=03 Prnt=41 Port=03 Cnt=01 Dev#= 42 Spd=12  MxCh= 0\
D:  Ver= 2.00 Cls=00(>ifc ) Sub=00 Prot=00 MxPS= 8 #Cfgs=  1\
P:  Vendor=0403 ProdID=6001 Rev=06.00\
S:  Manufacturer=FTDI\
S:  Product=US232R\
S:  SerialNumber=FTDBHXND\
C:  #Ifs= 1 Cfg#= 1 Atr=a0 MxPwr=100mA\
I:  If#=0x0 Alt= 0 #EPs= 2 Cls=ff(vend.) Sub=ff Prot=ff Driver=ftdi_sio
## Power tool unit (PTU)
T:  Bus=01 Lev=03 Prnt=41 Port=00 Cnt=01 Dev#= 45 Spd=12  MxCh= 0\
D:  Ver= 2.00 Cls=00(>ifc ) Sub=00 Prot=00 MxPS=64 #Cfgs=  1\
P:  Vendor=239a ProdID=80ce Rev=01.00\
S:  Manufacturer=Adafruit Industries LLC\
S:  Product=Feather M4 CAN\
S:  SerialNumber=591DCDD453393353202020360A1C11FF\
C:  #Ifs= 6 Cfg#= 1 Atr=80 MxPwr=100mA\
I:  If#=0x0 Alt= 0 #EPs= 1 Cls=02(commc) Sub=02 Prot=00 Driver=cdc_acm\
I:  If#=0x1 Alt= 0 #EPs= 2 Cls=0a(data ) Sub=00 Prot=00 Driver=cdc_acm\
I:  If#=0x2 Alt= 0 #EPs= 2 Cls=08(stor.) Sub=06 Prot=50 Driver=usb-storage\
I:  If#=0x3 Alt= 0 #EPs= 2 Cls=03(HID  ) Sub=00 Prot=00 Driver=usbhid\
I:  If#=0x4 Alt= 0 #EPs= 0 Cls=01(audio) Sub=01 Prot=00 Driver=snd-usb-audio\
I:  If#=0x5 Alt= 0 #EPs= 2 Cls=01(audio) Sub=03 Prot=00 Driver=snd-usb-audio
## Xsens inclinometer
T:  Bus=01 Lev=02 Prnt=40 Port=00 Cnt=01 Dev#= 44 Spd=12  MxCh= 0\
D:  Ver= 1.10 Cls=02(commc) Sub=00 Prot=00 MxPS= 8 #Cfgs=  1\
P:  Vendor=2639 ProdID=0013 Rev=00.00\
S:  Manufacturer=Xsens\
S:  Product=MTi-300 AHRS\
S:  SerialNumber=03782A7F\
C:  #Ifs= 2 Cfg#= 1 Atr=80 MxPwr=200mA\
I:  If#=0x0 Alt= 0 #EPs= 1 Cls=ff(vend.) Sub=00 Prot=00 Driver=(none)\
I:  If#=0x1 Alt= 0 #EPs= 2 Cls=ff(vend.) Sub=00 Prot=00 Driver=xsens_mt
# Instalation
* Copy the rules from this folder to the "/etc/udev/rules.d" folder.
* Run in terminal *"sudo udevadm trigger"* to reload the rules.
* Run in terminal *"ls -l /dev"* to check the mappings.

# Literature
Ubuntu: https://manpages.ubuntu.com/manpages/jammy/man7/udev.7.html 

Robot: https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html


