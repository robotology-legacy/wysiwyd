
from Tkinter import *

from drive_control_rpc import pointing, tagging, reset_all, freeze_all, unfreeze_all, narrate, manual_mode, automatic_mode

window = Tk()

def test():
    print "test"

def test2():
    print "test2"

title = Label(window, text="Manual drive control")

title.pack()

buttons = {}

buttons["Manual mode"] = Button(window, text="Manual mode", command=manual_mode)
buttons["Autonomous mode"] = Button(window, text="Autonomous mode", command=automatic_mode)
buttons["pointing"] = Button(window, text="Pointing", command=pointing)
buttons["tagging"] = Button(window, text="Tagging", command=tagging)
buttons["narate"] = Button(window, text="narrate", command=narrate)
buttons["freeze"] = Button(window, text="freeze", command=freeze_all)
buttons["unfreeze"] = Button(window, text="unfreeze", command=unfreeze_all)
buttons["reset"] = Button(window, text="reset", command=reset_all)


for b in buttons.values():
    b.pack()

window.mainloop()