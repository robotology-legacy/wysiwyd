#!/usr/bin/python

import urllib2
import base64
import time
import yarp
from pprint import pprint
import Image

import requests

# Initialise YARP
yarp.Network.init()

# Preparing port to connect to ABM
abm_port = yarp.RpcClient()
abm_local = '/sightcorp/abm:o'
abm_remote = '/autobiographicalMemory/rpc'
abm_port.open(abm_local)

# Loop to connect to ABM
while(not yarp.Network.connect(abm_local, abm_remote)):
    print("Waiting for connection to ABM")
    time.sleep( 1 )

# Ask where the images are stored
cmd = yarp.Bottle()
resp = yarp.Bottle()
cmd.addString('getStoringPath')
abm_port.write(cmd, resp)
storing_path = resp.get(0).asString()
print("Storing path: ", storing_path)

################################################# TO BE REMOVED ##################################################
storing_path = '/home/maxime/Downloads/Max.jpg'
storing_label = 'Max_label'
print("Fix storing path for testing purpose: ", storing_path)
################################################# TO BE REMOVED ##################################################

try:
    rpc_port = yarp.RpcServer()
    rpc_port.open("/sightcorp/rpc")

    while True:
        print("Waiting for command")
        cmd = yarp.Bottle()
        resp = yarp.Bottle()
        rpc_port.read(cmd,True)
        print("Got message: ", cmd.toString())
        if(cmd.get(0).asString() == 'tag'):
            print("Going to tag image")
            relative_path = cmd.get(1).asString()
            print("Relative path: ", relative_path)
            full_path = storing_path + '/' + relative_path
            print("Full path: ", full_path)
            #convert tif to jpg
            image_original = Image.open(full_path)
            image_original.save('/tmp/temp.jpg')



	    response_tagging = requests.post( 'http://api.sightcorp.com/api/detect/',
              data   = { 'app_key'   : 'your app_key',
                         'client_id' : 'your client_id' },
              files  = { 'img'       : ( storing_label, open( storing_path, 'rb' ) ) } )

            print("Result tagging:", response_tagging)

            if response_tagging.results is not None:
                tags = response_tagging.results[0].tags
                best_tag_name = tags[0].tag
                best_tag_confidence = tags[0].confidence

                resp.addString(best_tag_name)
                resp.addDouble(best_tag_confidence)
            else:
                resp.addString('nack')
                resp.addString('Did not get any results!')
        else:
            print("Not a valid command, abort")
            resp.addString("nack")

        rpc_port.reply(resp)

except urllib2.HTTPError, e:
    print('[HTTP Error {}]: {}'.format(e.code, e.reason))
    print('Request URL: {}'.format(e.geturl()))
    print('Response body: {}'.format(e.read()))
except urllib2.URLError, e:
    print('[URL Error]: {}'.format(e.reason))
except (KeyboardInterrupt, SystemExit):
    pass
except Exception, e:
    print(e)
finally:
    abm_port.close()
    rpc_port.close()
    yarp.Network.fini()


