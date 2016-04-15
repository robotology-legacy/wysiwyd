#!/usr/bin/env python

from collections import OrderedDict
from Tkinter import *

from drive_control_rpc import pointing, tagging, dummy, reset_all, freeze_all, unfreeze_all, narrate, manual_mode, automatic_mode

window = Tk()

def test():
    print "test"

def test2():
    print "test2"

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
buttons["narate"] = Button(window, text="Narrate", command=narrate)
buttons["dummy"] = Button(window, text="Dummy", command=dummy)


for b in buttons.values():
    b.pack()

window.mainloop()