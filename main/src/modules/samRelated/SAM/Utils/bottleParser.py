import sys
import time
import os

try:
	import yarp
	yarpRunning = True
except ImportError:
	print 'WARNING! Yarp was not found! Switching to offline mode'
	yarpRunning = False

def readBottle(bottle, resp='', indentation = 0):
	if(resp == None):
		resp = ''

	if(bottle !=  None):
		print 'bottle here'
		for i in range(bottle.size()):
			for j in range(indentation):
				resp += '\t'
			resp +=  '[' + str(i) + '] '
			val = bottle.get(i)
			code = val.getCode()
			if(code == yarp.BOTTLE_TAG_INT):
				resp += 'int (' + str(val.asInt()) + ')\n'
			elif(code == yarp.BOTTLE_TAG_DOUBLE):
				resp += 'double (' + str(val.asDouble()) + ')\n'
			elif(code == yarp.BOTTLE_TAG_STRING):
				resp += 'string (' + val.asString() + ')\n'
			elif(code == yarp.BOTTLE_TAG_BLOB):
				resp += 'blob of lenght (' + str(val.asBlobLength()) + ')\n'
			elif(val.isList()):
				resp += 'List of ' + str(val.asList().size()) + ' elements\n'
				indentation += 1
				resp = readBottle(val.asList(), resp, indentation)
				indentation -= 1
			else:
				resp += 'Unrecognized type\n'
	else:
		print 'no bottle'
	return resp

yarp.Network.init()

messagePort = yarp.RpcServer()
messagePort.open('/testServer')
inputBottle = yarp.Bottle()

while( True ):
	try:
		print 'Waiting for input:'
		messagePort.read(inputBottle,True)
		#parse Bottle Contents
		resp = readBottle(inputBottle)
		print resp
		print
	except KeyboardInterrupt:
		print 'Closing server ...'
		messagePort.close()
		try:
			sys.exit(0)
		except SystemExit:
			os._exit(0)
