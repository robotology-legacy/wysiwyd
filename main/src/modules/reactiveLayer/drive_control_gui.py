#!/usr/bin/env python

from collections import OrderedDict
from Tkinter import *

from drive_control_rpc import pointing, tagging, reset_all, freeze_all, unfreeze_all, narrate, manual_mode, automatic_mode, close_ports

window = Tk()

def on_closing():
    close_ports()
    window.destroy()

title = Label(window, text="Manual drive control")

title.pack()

buttons = OrderedDict()

buttons["Manual mode"] = Button(window, text="Manual mode", command=manual_mode)
buttons["Autonomous mode"] = Button(window, text="Autonomous mode", command=automatic_mode)
buttons["freeze"] = Button(window, text="Freeze", command=freeze_all)
buttons["unfreeze"] = Button(window, text="Unfreeze", command=unfreeze_all)
buttons["reset"] = Button(window, text="Reset", command=reset_all)
buttons["pointing"] = Button(window, text="Pointing", command=pointing)
buttons["tagging"] = Button(window, text="Tagging", command=tagging)
buttons["narrate"] = Button(window, text="Narrate", command=narrate)


for b in buttons.values():
    b.pack()

window.protocol("WM_DELETE_WINDOW", on_closing)
window.mainloop()