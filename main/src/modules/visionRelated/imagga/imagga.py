import urllib2
import base64
import time
import Imagga
import yarp
from pprint import pprint
import Image

# Initialise YARP
yarp.Network.init()

# initialize the API client
api_client = Imagga.swagger.ApiClient(api_server="https://api.imagga.com/v1")
# authentication setting using user name and password
api_client.username = 'acc_72743139adf67db'
api_client.password = '19c58b064598bde9fcb872268a0dc7c6'

abm_port = yarp.RpcClient()
abm_local = '/imagga/abm:o'
abm_remote = '/autobiographicalMemory/rpc'
abm_port.open(abm_local)

while(not yarp.Network.connect(abm_local, abm_remote)):
    print("Waiting for connection to ABM")
    time.sleep( 1 )

cmd = yarp.Bottle()
resp = yarp.Bottle()
cmd.addString('getStoringPath')
abm_port.write(cmd, resp)
storing_path = resp.get(0).asString()
print("Storing path: ", storing_path)

try:
    rpc_port = yarp.RpcServer()
    rpc_port.open("/imagga/rpc")

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

            content_api = Imagga.ContentApi(api_client)
            tagging_api = Imagga.TaggingApi(api_client)

            response_upload = content_api.upload('/tmp/temp.jpg')
            pprint(response_upload)
            content_id = response_upload.uploaded[0].id

            response_tagging = tagging_api.tagging(content=content_id)
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

