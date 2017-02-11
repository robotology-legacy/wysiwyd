#!/usr/bin/env python

from collections import OrderedDict
from Tkinter import *
import sys, getopt
argv = sys.argv[1:]

x = 40
y = 40
h = 200
w = 400

try:
  opts, args = getopt.getopt(argv,"w:h:x:y:",["width=","height=","x=","y="])
except getopt.GetoptError:
  print 'drive_control_gui -w <width> -h <height> -x <x> -y <y>'
  sys.exit(2)
for opt, arg in opts:
  if opt in ('-h', '--height'):
    h=int(arg)
  elif opt in ('-w', '--weight'):
    w=int(arg)
  elif opt in ('-x'):
    x=int(arg)
  elif opt in ('-y'):
    y=int(arg)
    

from drive_control_rpc import pointing, tagging, reset_all, freeze_all, unfreeze_all, narrate, manual_mode, automatic_mode, close_ports, updateDriveList, trigger_behavior, updateBehaviorList

window = Tk()

def on_closing():
    window.destroy()
    close_ports()

def updateDriveButtons():
    for button in driveButtons.keys():

        driveButtons[button].destroy()
    driveList = updateDriveList()
    for i,drive in enumerate(driveList):
        driveButtons[drive]=Button(window, text=drive, command=lambda: trigger_behavior(drive))
        driveButtons[drive].grid(row=i+3, column=1)
def updateBehaviorButtons():
    for button in behaviorButtons.keys():

        behaviorButtons[button].destroy()
    behaviorList = updateBehaviorList()
    for i,beh in enumerate(behaviorList):
        behaviorButtons[beh]=Button(window, text=beh, command=lambda: trigger_behavior(beh))
        behaviorButtons[beh].grid(row=i+3, column=2)
def please():
    print 'Pleaes Update'
title = Label(window, text="Manual drive control").grid(row=0,column=1)

labels = OrderedDict()

labels["General Control:"] = Label(window,text="General Control:").grid(row=1,column=0)
labels["Drive Control:"] = Label(window,text="Drive Control:").grid(row=1,column=1)
labels["Behavior Control:"] = Label(window,text="Behavior Control:").grid(row=1,column=2)



fixed_buttons = OrderedDict()
update_buttons = OrderedDict()
driveButtons = OrderedDict()
behaviorButtons = OrderedDict()
update_buttons["Update drive list"] = Button(window, text="Update list", command=updateDriveButtons).grid(row=2,column=1)
update_buttons["Update behavior list"] = Button(window, text="Update list", command=updateBehaviorButtons).grid(row=2,column=2)

fixed_buttons["Manual mode"] = Button(window, text="Manual mode", command=manual_mode)
fixed_buttons["Autonomous mode"] = Button(window, text="Autonomous mode", command=automatic_mode)
fixed_buttons["freeze"] = Button(window, text="Freeze", command=freeze_all)
fixed_buttons["unfreeze"] = Button(window, text="Unfreeze", command=unfreeze_all)
fixed_buttons["reset"] = Button(window, text="Reset", command=reset_all)
#driveButtons["pointing"] = Button(window, text="Pointing", command=pointing)
#driveButtons["tagging"] = Button(window, text="Tagging", command=tagging)
driveButtons["dummyButtonD"] = Button(window, text="dummyButton", command=please)
behaviorButtons["dummyButtonB"] = Button(window, text="dummyButton", command=please)



for i,b in enumerate(fixed_buttons.values()):
    b.grid(row=i+2, column=0)

for i,b in enumerate(driveButtons.values()):
    b.grid(row=i+3, column=1)
for i,b in enumerate(behaviorButtons.values()):
    b.grid(row=i+3, column=2)

window.geometry('%dx%d+%d+%d' % (w, h, x, y))
window.protocol("WM_DELETE_WINDOW", on_closing)
window.mainloop()