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
labelStorePath = '/home/daniel/WYSIWYD_PROJECT/actionRecognitionDataset/push-pull-right_arm/labels/data.log'
dataStorePath = '/home/daniel/WYSIWYD_PROJECT/actionRecognitionDataset/push-pull-right_arm/data/data.log'

pygame.init()
screen = pygame.display.set_mode((640, 100))
pygame.display.set_caption('Dataset Labeller')
pygame.mouse.set_visible(1)
pygame.key.set_repeat(100,100)

yarp.Network.init()

messagePort = yarp.RpcClient()
messagePort.open(clientName)

dataPort = yarp.BufferedPortBottle()
dataPort.open(dataPortName)

connected = False
step = False
inputBottle = yarp.Bottle()
outputBottle = yarp.Bottle()
dataInBottle = yarp.Bottle()
pressed = False
seclabel = ''
previousIdx = 0
currIdx = 0
timeString = ''

dataFile = open(dataStorePath,'r')
count = 0
dataList = []

for line in dataFile:
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
			dataInBottle.clear()
			reads = dataPort.getPendingReads()
			if(reads > 0):
				dataPort.read(dataInBottle)
				#print dataInBottle.toString()
				dataPort.getEnvelope(dataInBottle)
				timeString = dataInBottle.toString()

			events = pygame.event.get()
			for event in events:
				if event.type == pygame.KEYDOWN:
					if (event.key == pygame.K_RETURN):
						pressed = True
					elif(event.key == pygame.K_SPACE):
						step = True
			if(pressed):
				outputBottle = yarp.Bottle('pause')
				print 'Paused...\n'
				messagePort.write(outputBottle, inputBottle)

				idx = int(timeString.split(' ')[0])
				currIdx = idx

				#get label of past action
				sectionLabel = ask(screen, "Section Label")
				print 'Assigning ' + sectionLabel + ' to timestamp range from ' + dataList[previousIdx][1] + ' to ' + dataList[currIdx][1]
				print 'Assigning ' + sectionLabel + ' to index range from ' + dataList[previousIdx][0] + ' to ' + dataList[currIdx][0]

				#fill range between last and curr index with message
				sectionLabel = '(' + sectionLabel + ')'
				for i in range(previousIdx, currIdx+1):
					if(len(dataList[i]) > 2):
						dataList[i][2] = sectionLabel
					else:
						dataList[i].append(sectionLabel)

				previousIdx = currIdx

				outputBottle = yarp.Bottle('play')
				print 'Continue...\n'
				messagePort.write(outputBottle, inputBottle)

				pressed = False
				step = True
			elif(step):
				outputBottle = yarp.Bottle('play')
				print 'Time before step: ' + timeString
				messagePort.write(outputBottle, inputBottle)

				reads = 0
				while(reads == 0):
					reads = dataPort.getPendingReads()

				outputBottle = yarp.Bottle('pause')
				messagePort.write(outputBottle, inputBottle)
				step = False;


except KeyboardInterrupt:
	print
	print 'Interrupted'
	dataString =''
	for i in range(len(dataList)):
		dataString += " ".join(dataList[i]) + "\n"
	labelFile = open(labelStorePath,'w')
	labelFile.write(dataString)
	labelFile.close()
	dataPort.close()
	messagePort.close()
	try:
		sys.exit(0)
	except SystemExit:
		os._exit(0)

