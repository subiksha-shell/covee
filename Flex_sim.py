import numpy as np 
import csv
import os
import coloredlogs, logging, threading
from submodules.dmu.dmu import dmu
from submodules.dmu.httpSrv import httpSrv
from submodules.dmu.mqttClient import mqttClient
import time
import sys
import requests
import json
import csv

coloredlogs.install(level='DEBUG',
fmt='%(asctime)s %(levelname)-8s %(name)s[%(process)d] %(message)s',
field_styles=dict(
    asctime=dict(color='green'),
    hostname=dict(color='magenta'),
    levelname=dict(color='white', bold=True),
    programname=dict(color='cyan'),
    name=dict(color='blue')))
logging.info("Program Start")

if bool(os.getenv('MQTT_ENABLED')):
    mqtt_url = str(os.getenv('MQTTURL'))
    mqtt_port = int(os.getenv('MQTTPORT'))
    mqtt_user = str(os.getenv('MQTTUSER'))
    mqtt_password = str(os.getenv('MQTTPASS'))
else:
    mqtt_url = "mqtt"
    mqtt_port = 1883
    mqtt_password = ""
    mqtt_user = ""

############################ Start the Server #######################################################

''' Initialize objects '''
dmuObj = dmu()

''' Start mqtt client '''
mqttObj = mqttClient(mqtt_url, dmuObj, mqtt_port, mqtt_user, mqtt_password)

time.sleep(2.0)
#######################################################################################################

flex_offer = {"variables": [],"flex_output": {}}
flex_request_dict = {}

# Receive flexibility_request
dmuObj.addElm("flex_values", {})
mqttObj.attachSubscriber("/edgeflex/goflex/flex_request","json","flex_values")

# Send flexibility_output
dmuObj.addElm("flex_input", {})
mqttObj.attachPublisher("/edgeflex/goflex/flex_result","json","flex_input")

while True:
    flex_request = dmuObj.getDataSubset("flex_values")
    if flex_request:
        flex_output = {}
        print(flex_request)
        for i in flex_request["variables"]:
            if flex_request["flex_dict"][i]:
                logging.info("flexibility request " + str(i) + " = " + str(flex_request["flex_dict"][i]))
                logging.info(" ")
        time.sleep(2)
        ###################### PREPARE THE ANSWER ###########
        flex_output = {"variable":["active_power"],"active_power": {"vector_position":[2,3],"prediction": 1,"flex_value": [0.2,0.2]}}
        print(flex_output)
        dmuObj.setDataSubset(flex_output,"flex_input")
        time.sleep(0.5)
        break
        
