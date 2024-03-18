# -*- coding: utf-8 -*-
"""
Created on Tue Aug  2 17:42:28 2022


Bohrhammer-Betätigung DEMO-Mode:
Für die Bohrhammer-Betätigung ON/OFF ist eine Option mi Checkbox/Schalter und 
eine zweite mit Push-Button vorgesehen (diese setzt Taster wieder zurück).
TKinter bietet keine schöne Möglichkeit für runde Knöpfe (Tastfunktion) darum
ästhetisch wenig ansprechende Umsetzung.

Eine initiale Umsetzung mit permanentem Schreiben der Ansteuerungen auf die serielle 
Schnittstelle erwies sich als wenig performant (Reaktionszeit). Darum eine event-basierte
Umsetzung, die die Aktionen an Betätigungsevents bindet.
Auch die Benutzung der "after" Methode von Tkinter (mit 100ms) erwies sich als viel zu langsam.

Darum wurde für diese Demon ein "klassischer" Ansatz mit einer Klasse für jedes GUI-Element benutzt.
Ein Umsetzung des Einlesens von Sensorsignale ist in dieser Version nicht implementiert.

Positionierung mit der PLACE-Option ist für verschiedene Screens schwierig bzw.
macht diese unleserlich/verzerrt. Initial wg. einfachem Handling aber damit gestartet.
Die vermutlich geeigneteren Positionierungsoptionen für verschiedene Displaygrößen 
sind GRID oder PACK


Multi-Threading ist mit HMI/GUI-Anforderungen praktisch nicht möglich (meinst nur eine Main-Task) zulässig.
Darum wird auch die serielle Kommunikation in das main integriert.  


Umsetzung der permanenten Ausführung über WHILE-TRUE mit TKinter nicht möglich,
hier Umsetzung über Methode .after.
Code muss noch MAIN und bessere Struktur bekommen, außerdem ist unklar, wie zuverlässig
die "after" Methode ist. Multithreading-Ansatz war auch nicht performant

ToDo:
- Anordnung der Elemente
- Taster-Betätigung: Soll evtl. gedrückte Toggle-Funktion abschalten und Bild aktualisieren
- Serielle Elemente in Klasse "verpacken"

@author: DIE2SI
"""

import tkinter as tk
import time
from tkinter import *
import serial


class BohrhammerOnOff(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.chkValue = tk.BooleanVar() 
        self.image = PhotoImage(file = 'images/OnHammer_left_196.png') 
        self.selectimage = PhotoImage(file = 'images/OffHammer_right_196.png')        
        self.button = tk.Checkbutton(self, text="Bohrhammer ON/OFF", var=self.chkValue, image=self.image, selectimage=self.selectimage)
        self.text = tk.Text(self, width=40, height=6)
        self.vsb = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.vsb.set)


        self.button.pack(side="top")
        self.vsb.pack(side="right", fill="y")
        self.text.pack(side="bottom", fill="x")

        self.button.bind("<ButtonPress>", self.on_press)
        self.button.bind("<ButtonRelease>", self.on_release)

    def on_press(self, event):
        self.log("Schalter button was pressed")
        self.log('Schalter button with state' + str(self.chkValue.get()))
        if self.chkValue.get():
            print('Schalter OFF')
            ser.write(b'D5_Off\n')
        else:
            print('Schalter ON')        
            ser.write(b'D5_On\n')    

    def on_release(self, event):
        self.log("Schalter button was released")

    def log(self, message):
        now = time.strftime("%I:%M:%S", time.localtime())
        self.text.insert("end", now + " " + message.strip() + "\n")
        self.text.see("end")

class BohrhammerTaster(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.image = PhotoImage(file = 'images/OnHammer_100_round.png')
        self.button = tk.Button(self, text="Bohrhammer Taster", image=self.image)
        self.text = tk.Text(self, width=40, height=6)
        self.vsb = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.vsb.set)

        self.button.pack(side="top")
        self.vsb.pack(side="right", fill="y")
        self.text.pack(side="bottom", fill="x")

        self.button.bind("<ButtonPress>", self.on_press)
        self.button.bind("<ButtonRelease>", self.on_release)

    def on_press(self, event):
        self.log("Taster button was pressed")
        ser.write(b'D5_On\n')

    def on_release(self, event):
        self.log("Taster button was released")
        ser.write(b'D5_Off\n')

    def log(self, message):
        now = time.strftime("%I:%M:%S", time.localtime())
        self.text.insert("end", now + " " + message.strip() + "\n")
        self.text.see("end")


class BohrhammerRL(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.chkValue = tk.BooleanVar() 
        self.image = PhotoImage(file = 'images/Right_100.png') 
        self.selectimage = PhotoImage(file = 'images/Left_100.png')        
        self.button = tk.Checkbutton(self, text="Bohrhammer R/L", var=self.chkValue, image=self.image, selectimage=self.selectimage)
        self.text = tk.Text(self, width=40, height=6)
        self.vsb = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.vsb.set)

        self.button.pack(side="top")
        self.vsb.pack(side="right", fill="y")
        self.text.pack(side="bottom", fill="x")

        self.button.bind("<ButtonPress>", self.on_press)
        self.button.bind("<ButtonRelease>", self.on_release)

    def on_press(self, event):
        self.log("R/L button was pressed")
        self.log('R/L button with state' + str(self.chkValue.get()))
        if self.chkValue.get():
            print('R/L Schalter OFF')
            ser.write(b'D6_L\n')
        else:
            print('R/L Schalter ON')      
            ser.write(b'D6_R\n')

    def on_release(self, event):
        self.log("R/L button was released")

    def log(self, message):
        now = time.strftime("%I:%M:%S", time.localtime())
        self.text.insert("end", now + " " + message.strip() + "\n")
        self.text.see("end")

# Creating label class (contains label for RPM-title)
class BohrhammerLabelRPM:
    def __init__(self, master) -> None:
        # Instantiating master i.e toplevel Widget
        self.master = master

        # Creating first Label i.e with default font-size
        Label(self.master, text="Eingabe Drehzahl [0..100 %]", font=("Arial", 18)).pack(pady=10)


class BohrhammerRPM(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.value = ""
        self.entry = tk.Entry(text='RPM-Sollwert')#(self.root)
        self.entry.bind("<Return>", self.onReturn)
        self.entry.pack()

    def onReturn(self, event):
        print("Return Pressed")
        self.value = self.entry.get()
        print(self.value)
        # Abfrage leerer String, weil das bei Eingabe vorkommen kann --> dann Ausgabe von 0
        if self.value =='':
            val = 0
            ser.write(b'A0=0\n')
        else:
            val = float(self.value) 
            ser.write(b'A0=' + str(val).encode('utf-8') +'\n'.encode('utf-8'))
        self.entry.delete(0,'end')


if __name__ == "__main__":
    # configure the serial connections (the parameters differs on the device you 
    # are connected to, e.g. USB-COM may change depending on connected items)
    # To Adafruit
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

    # configure of GUI
    bohrhammerGUI = tk.Tk()
    BohrhammerOnOff(bohrhammerGUI).pack(side="top", fill="both", expand=True)
    BohrhammerTaster(bohrhammerGUI).pack(side="top", fill="both", expand=True)
    BohrhammerRL(bohrhammerGUI).pack(side="top", fill="both", expand=True)
    BohrhammerLabelRPM(bohrhammerGUI)
    BohrhammerRPM(bohrhammerGUI).pack(side="top", fill="both", expand=True)
    bohrhammerGUI.mainloop()

# Workaround for stopping Toggle when push-button is activated
if BohrhammerOnOff(bohrhammerGUI).getvar() and BohrhammerTaster(bohrhammerGUI).on_press:
    BohrhammerOnOff(bohrhammerGUI).setvar = False