#!/usr/bin/env python

from collections import OrderedDict
from Tkinter import *

from drive_control_rpc import pointing, tagging, reset_all, freeze_all, unfreeze_all, narrate, manual_mode, automatic_mode, close_ports, updateDriveList, trigger_behavior

window = Tk()

def on_closing():
    close_ports()
    window.destroy()

def updateButtons():
    for button in buttons.keys():

        buttons[button].destroy()
    driveList = updateDriveList()
    for drive in driveList:
        buttons[drive]=Button(window, text=drive, command=lambda: trigger_behavior(drive))
        buttons[drive].pack()

title = Label(window, text="Manual drive control")

title.pack()

fixed_buttons = OrderedDict()
buttons = OrderedDict()
fixed_buttons["Update list"] = Button(window, text="Update list", command=updateButtons)

fixed_buttons["Manual mode"] = Button(window, text="Manual mode", command=manual_mode)
fixed_buttons["Autonomous mode"] = Button(window, text="Autonomous mode", command=automatic_mode)
fixed_buttons["freeze"] = Button(window, text="Freeze", command=freeze_all)
fixed_buttons["unfreeze"] = Button(window, text="Unfreeze", command=unfreeze_all)
fixed_buttons["reset"] = Button(window, text="Reset", command=reset_all)
buttons["pointing"] = Button(window, text="Pointing", command=pointing)
buttons["tagging"] = Button(window, text="Tagging", command=tagging)
buttons["narrate"] = Button(window, text="Narrate", command=narrate)



for b in fixed_buttons.values():
    b.pack()

for b in buttons.values():
    b.pack()
window.protocol("WM_DELETE_WINDOW", on_closing)
window.mainloop()