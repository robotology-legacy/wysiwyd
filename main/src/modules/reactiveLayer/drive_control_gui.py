
from Tkinter import *

from drive_control_rpc import pointing, tagging, reset_all, freeze_all, unfreeze_all

window = Tk()

def test():
    print "test"

def test2():
    print "test2"

title = Label(window, text="Manual drive control")

title.pack()

buttons = {}

buttons["Manual mode"] = Button(window, text="Manual mode", command=freeze_all)
buttons["Autonomous mode"] = Button(window, text="Manual mode", command=unfreeze_all)
buttons["pointing"] = Button(window, text="Pointing", command=pointing)
buttons["tagging"] = Button(window, text="Tagging", command=tagging)


for b in buttons.values():
    b.pack()

window.mainloop()