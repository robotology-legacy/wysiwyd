import sys
import time
import os
import pygame
import string
from pygame import font, event, draw
from pygame.locals import *
import yarp

def get_key():
  while 1:
    event = pygame.event.poll()
    if event.type == pygame.KEYDOWN:
      return event.key
    else:
      pass

def display_box(screen, message):
  "Print a message in a box in the middle of the screen"
  fontobject = pygame.font.Font(None,25)
  w = 300
  h = 20
  pygame.draw.rect(screen, (0,0,0),
                   ((screen.get_width() / 2) - w,
                    (screen.get_height() / 2) - h,
                    w*2,h*2), 0)
  pygame.draw.rect(screen, (255,255,255),
                   ((screen.get_width() / 2) - w - 2,
                    (screen.get_height() / 2) - h - 2,
                    w*2+2,h*2+2), 1)
  if len(message) != 0:
    screen.blit(fontobject.render(message, 1, (255,255,255)),
                ((screen.get_width() / 2) - w, (screen.get_height() / 2) - h))
  pygame.display.flip()

def ask(screen, question):
  #"ask(screen, question) -> answer"
  current_string = []
  display_box(screen, question + ": " + string.join(current_string,""))
  while 1:
    inkey = get_key()
    if inkey == K_BACKSPACE:
      current_string = current_string[0:-1]
    elif inkey == K_RETURN:
      break
    elif inkey == K_MINUS:
      current_string.append("_")
    elif inkey <= 127:
      current_string.append(chr(inkey))
    display_box(screen, question + ": " + string.join(current_string,""))
  return string.join(current_string,"")

clientName = '/testSender'
dataPortName = '/dataPort'
serverName = '/yarpdataplayer/rpc:i'
labelStorePath = '/home/daniel/WYSIWYD_PROJECT/actionRecognitionDataset/lift-drop-left_arm/labels/data.log'
dataStorePath = '/home/daniel/WYSIWYD_PROJECT/actionRecognitionDataset/lift-drop-left_arm/data/data.log'

pygame.init()
screen = pygame.display.set_mode((640, 100))
pygame.display.set_caption('Dataset Labeller')
pygame.mouse.set_visible(1)

yarp.Network.init()

messagePort = yarp.RpcClient()
messagePort.open(clientName)

dataPort = yarp.Port()
dataPort.open(dataPortName)

connected = False
inputBottle = yarp.Bottle()
outputBottle = yarp.Bottle()
dataInBottle = yarp.Bottle()
pressed = False
seclabel = ''
previousTimeStamp = 0
currTimeStamp = 0

dataFile = open(dataStorePath,'r')
count = 0
dataList = []
logList = []

for line in dataFile:
	logList.append(line)
	t = line.split(' ')[1]
	dataList.append([str(count), t])
	count += 1
dataFile.close()

try:
	while(True):
		if(messagePort.getOutputCount() == 0):
			print 'Waiting for a connection'
			print str(messagePort.getOutputCount())
			yarp.Network.connect(clientName, serverName)
			time.sleep(1)
		else:
			events = pygame.event.get()
			for event in events:
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_RETURN:
						pressed = True
			if(pressed):
				outputBottle = yarp.Bottle('pause')
				print 'Paused...\n'
				messagePort.write(outputBottle, inputBottle)

				dataInBottle.clear()
				dataPort.read(dataInBottle)
				#print getEnvelope(dataInBottle)
				#get curent timestamp
				print dataInBottle.toString()

				dataPort.getEnvelope(dataInBottle)
				print dataInBottle.toString()

				dataPort.getEnvelope(dataInBottle)
				print dataInBottle.toString()

				#t = dataInBottle.toString().split(' ')[1]
				#indx = [s for s in dataList if t in s]
				#ind = indx[0][0]
				t = dataInBottle.toString()
				print t


				#get label of past action
				sectionLabel = ask(screen, "Section Label")
				print sectionLabel

				#fill range between last and curr index with message
				# dataList[count].append('hello')

				outputBottle = yarp.Bottle('play')
				print 'Continue...\n'
				messagePort.write(outputBottle, inputBottle)

				pressed = False

except KeyboardInterrupt:
	print
	print 'Interrupted'
	dataString =''
	for i in range(len(logList)):
		dataString += " ".join(logList[i]) + os.linesep
	labelFile = open(labelStorePath,'w')
	labelFile.write(dataString)
	labelFile.close()
	messagePort.close()
	try:
		sys.exit(0)
	except SystemExit:
		os._exit(0)

