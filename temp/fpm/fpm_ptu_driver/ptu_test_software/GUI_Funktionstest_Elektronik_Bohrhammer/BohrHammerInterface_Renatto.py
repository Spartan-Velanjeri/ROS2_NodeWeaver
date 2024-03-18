from cgitb import text
from doctest import master
from email.mime import image
import tkinter as tk
import tkinter.messagebox
import customtkinter
from datetime import datetime
import serial

customtkinter.set_appearance_mode("Light")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("dark-blue")  # Themes: "blue" (standard), "green", "dark-blue"

class App(customtkinter.CTk):

    WIDTH = 780
    HEIGHT = 520
    bohrhammer_on = False
    drill_on = False
    direction = True    #True: Right, False: Left
    

    def __init__(self):
        super().__init__()
        print("GUI initiated at {0}".format(datetime.now()))

        self.title("Bohrhammer Control")
        self.geometry(f"{App.WIDTH}x{App.HEIGHT}")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)  # call .on_closing() when app gets closed

        

        # configure grid layout (2x1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        
        self.frame_left = customtkinter.CTkFrame(master=self,
                                                 width=180,
                                                 corner_radius=0)
        self.frame_left.grid(row=0, column=0, sticky="nswe")

        self.frame_right = customtkinter.CTkFrame(master=self)
        self.frame_right.grid(row=0, column=1, sticky="nswe", padx=20, pady=20)

        #LEFT FRAME

        self.frame_left.grid_rowconfigure(0, minsize=50)   # empty row with minsize as spacing
        self.frame_left.grid_rowconfigure(2, minsize=50)   # empty row with minsize as spacing

        self.label_1 = customtkinter.CTkButton(master=self.frame_left,
                                              image = tkinter.PhotoImage(file='images/hammer.png'),
                                              text= 'Bohrhammer Control',
                                              text_font = 'Roboto',
                                              text_color= 'white',
                                              hover =  False,
                                              compound='top'
                                              )  # font name and size in px
        self.label_1.grid(row=1, column=0, pady=10, padx=10)

        self.on_off_button = customtkinter.CTkButton(master=self.frame_left,
                                                text="OFF",
                                                text_font = 'Roboto',
                                                fg_color = 'Red',
                                                hover_color='#FFCCCB',
                                                text_color= 'white',  
                                                command=self.onOff_event)

        
        self.on_off_button.grid(row=3, column=0, pady=10, padx=20, ipadx=10, ipady=10, sticky="nswe")

        self.changeDirection_button = customtkinter.CTkButton(master=self.frame_left,
                                                text="Right",
                                                text_font = 'Roboto',
                                                text_color= 'white',
                                                command=self.changeDirection_event)
        self.changeDirection_button.grid(row=4, column=0, pady=10, padx=20, ipadx=10, ipady=10, sticky="nswe")

        self.taster = customtkinter.CTkButton(master=self.frame_left,
                                                text="Taster",
                                                text_font = 'Roboto',
                                                fg_color='red',
                                                text_color= 'white',
                                                hover_color="#FFCCCB",
                                                state= tkinter.DISABLED,
                                                command=self.on_press
                                                )
        self.taster.grid(row=6, column=0, pady=10, padx=20, ipadx=10, ipady=10, sticky="nswe")


        self.label_info_1 = customtkinter.CTkButton(master=self.frame_right,
                                                   image = tkinter.PhotoImage(file='images/default_hammer.png'),
                                                   text= '',
                                                   height=300,
                                                   width=500,
                                                   corner_radius=6,
                                                   hover =  False  # <- custom corner radius
                                                   )
        self.label_info_1.grid(column=0, row=0, sticky="nwe", padx=15, pady=15)

        self.RPM_Label = customtkinter.CTkLabel(master=self.frame_right,
                                                    text="RPM Slider",
                                                    text_font = 'Roboto',
                                                    text_color= 'black',
                                                    )

        self.RPM_Label.grid(row=1, column=0, sticky="ew")

        self.progressbar = customtkinter.CTkSlider(master=self.frame_right,
                                                   command=self.slider,
                                                   from_=0,
                                                   to=500,
                                                   number_of_steps=500,)
        self.progressbar.grid(row=2, column=0, sticky="ew")

        self.RPM = customtkinter.CTkLabel(master=self.frame_right,
                                                    text="RPM = {0}".format(self.progressbar.get()),
                                                    text_font = 'Roboto',
                                                    text_color= 'black',
                                                    )

        self.RPM.grid(row=3, column=0, sticky="ew", pady=15)

    def slider(self,event):
        
        self.RPM.configure(text="RPM = {0}".format(self.progressbar.get()))
        self.com_RPM()

    def onOff_event(self):
        App.bohrhammer_on^=True
        if App.bohrhammer_on:
            self.on_off_button.configure(text='ON')
            self.on_off_button.configure(fg_color='green')
            self.on_off_button.configure(hover_color='#90EE90')
            self.taster.configure(state=tkinter.NORMAL)
            

        else:
            self.on_off_button.configure(text='OFF')
            self.on_off_button.configure(fg_color='red')
            self.on_off_button.configure(hover_color='#FFCCCB')
            App.drill_on=False
            self.taster.configure(state=tkinter.DISABLED)
            self.taster.configure(fg_color='red')
            self.taster.configure(hover_color='#FFCCCB')
            self.com_Off()
            
            
        print(self.on_off_button.text)


    def changeDirection_event(self):
        App.direction^=True
        if App.direction:
            #Right
            self.changeDirection_button.configure(text='Right')
        else:
            #Left
            self.changeDirection_button.configure(text='Left')
        self.com_direction()
    
    def on_press(self):
        App.drill_on^=True
        if App.drill_on:
            self.taster.configure(fg_color='green')
            self.taster.configure(hover_color='#90EE90')
            self.com_On()
        else:
            self.taster.configure(fg_color='red')
            self.taster.configure(hover_color='#FFCCCB')
            self.com_Off()



    
    def on_release(self):
        print("Button released")

    def change_appearance_mode(self, new_appearance_mode):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def on_closing(self, event=0):
        self.com_Off()
        self.destroy()

    def com_RPM(self):
        RPM = self.progressbar.get()
        print("RPM = {0}".format(RPM))
        #ser.write(b'A0=' + str(RPM/500).encode('utf-8') +'\n'.encode('utf-8'))
        ser.write(b'A0=' + str(100*(RPM/500)).encode('utf-8') +'\n'.encode('utf-8'))

    def com_direction(self):
        print("Current Direction {0}".format('Right' if App.direction else 'Left'))
        ser.write(b'D6_R\n') if App.direction else ser.write(b'D6_L\n')
    
    def com_On (self):
        print("Drilling")
        ser.write(b'D5_On\n')

    def com_Off (self):
        print("Driller Off")
        ser.write(b'D5_Off\n')



 


if __name__ == "__main__":
    ser = serial.Serial(
        port='COM9',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        xonxoff= False  
    )

    ser.close()
    print('main Closed?:', ser.is_open)

    ser.open()
    print('main Opened?:', ser.is_open)


    app = App()
    app.iconbitmap("images/myIcon.ico")
    try:
        app.mainloop()
    except:
        app.destroy()
    
    ser.close()
    print('main Closed?:', not ser.is_open)