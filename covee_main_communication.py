import numpy as np 
import os
import coloredlogs, logging, threading
from threading import Thread
from submodules.dmu.dmu import dmu
from submodules.dmu.httpSrv import httpSrv
from submodules.dmu.mqttClient import mqttClient
import time
import csv
import sys
import json
from pypower.api import *
from pypower.ext2int import ext2int
import control_strategies.utils as utils
from importlib import import_module

# Set Control Strategy and Case
##################################################################
with open("../conf.json", "r") as f:
    conf_dict = json.load(f)

module_obj = utils.select(conf_dict)
Quadratic_Control = module_obj.select_control()
# # Choose the case (maybe it can be a env variable)
sys.path.insert(0,'../')
from cases.case_10_nodes import case_10_nodes as case
sys.path.remove('../')
##################################################################

coloredlogs.install(level='DEBUG',
fmt='%(asctime)s %(levelname)-8s %(name)s[%(process)d] %(message)s',
field_styles=dict(
    asctime=dict(color='green'),
    hostname=dict(color='magenta'),
    levelname=dict(color='white', bold=True),
    programname=dict(color='cyan'),
    name=dict(color='blue')))
logging.info("Program Start")
#####################################################################


############################ Start the Server #######################################################

external_mqtt = conf_dict["EXT_MQTT"]
if bool(os.getenv('MQTT_ENABLED')) and external_mqtt == "yes":
    mqtt_url = str(os.getenv('MQTTURL'))
    mqtt_port = int(os.getenv('MQTTPORT'))
    mqtt_user = str(os.getenv('MQTTUSER'))
    mqtt_password = str(os.getenv('MQTTPASS'))
else:
    mqtt_url = "mqtt"
    mqtt_port = 1883
    mqtt_password = ""
    mqtt_user = ""

''' Initialize objects '''
dmuObj = dmu()

''' Start mqtt client '''
mqttObj = mqttClient(mqtt_url, dmuObj, mqtt_port, mqtt_user, mqtt_password)

''' Start http server '''
httpSrvThread2 = threading.Thread(name='httpSrv',target=httpSrv, args=("0.0.0.0", int(conf_dict["EXT_PORT_COVEE"]) ,dmuObj,))
httpSrvThread2.start()

time.sleep(2.0)
#######################################################################################################


########################################################################################################
#########################  Section for Defining the Dictionaries  ######################################
########################################################################################################

dict_ext_cntr = {
    "data_cntr" : [],
    "data_nodes" : [],
    "ESS_nodes" : []
}

simDict = {
    "active_nodes" : [],
    "voltage_node" : [],
    "pv_input_node" : [],
    "ESS_nodes" : []
}

voltage_dict = {}
active_power_dict = {}
reactive_power_dict = {}
active_power_ESS_dict = {}
pv_input_dict = {}
pmu_input = {}

# add the simulation dictionary to dmu object
dmuObj.addElm("simDict", simDict)
dmuObj.addElm("voltage_dict", voltage_dict)
dmuObj.addElm("active_power_dict", active_power_dict)
dmuObj.addElm("reactive_power_dict", reactive_power_dict)
dmuObj.addElm("pv_input_dict", pv_input_dict)
dmuObj.addElm("active_power_ESS_dict", active_power_ESS_dict)
dmuObj.addElm("pmu_input", pmu_input)

########################################################################################################
#########################  Section for Receiving Signal  ###############################################
########################################################################################################

def api_cntr_input(data, uuid, name,  *args):   
    tmpData = []
    logging.debug("RECEIVED EXTERNAL CONTROL")
    logging.debug(data)       
    dmuObj.setDataSubset(data["nodes"],"simDict", "active_nodes")
    dmuObj.setDataSubset(data["ESS"],"simDict", "ESS_nodes")

# Receive from external Control
dmuObj.addElm("nodes", dict_ext_cntr)
dmuObj.addElmMonitor(api_cntr_input, "nodes", "data_nodes")
# Receive voltage
dmuObj.addElm("voltage", simDict)
mqttObj.attachSubscriber("/voltage_control/measuremnts/voltage","json","voltage_dict")
# Receive pv_input
dmuObj.addElm("pv_input", simDict)
mqttObj.attachSubscriber("/voltage_control/measuremnts/pv","json","pv_input_dict")
# Receive pmu_meas
dmuObj.addElm("pmu_meas", {})
mqttObj.attachSubscriber("/edgeflex/edgepmu0/ch0","json","pmu_input")
########################################################################################################
#########################  Section for Sending Signal  #################################################
########################################################################################################

mqttObj.attachPublisher("/voltage_control/control/active_power","json","active_power_dict")
mqttObj.attachPublisher("/voltage_control/control/reactive_power","json","reactive_power_dict")
mqttObj.attachPublisher("/voltage_control/control/active_power_ESS","json","active_power_ESS_dict")


'''
#########################################################################################################
#########################################################################################################
#########################################################################################################
'''
ppc = case()
ppc = ext2int(ppc)      # convert to continuous indexing starting from 0
BUS_TYPE = 1
additional = utils.additional() 
[grid_data,reactive_power,active_power,active_power_ESS] = additional.system_info(ppc,BUS_TYPE, conf_dict["CONTROL_DATA"]["active_nodes"], conf_dict["CONTROL_DATA"]["active_ESS"])

active_nodes = conf_dict["CONTROL_DATA"]["active_nodes"] 
active_ESS = conf_dict["CONTROL_DATA"]["active_ESS"]
active_nodes_old = active_nodes
active_ESS_old = active_ESS
print("active_nodes", active_nodes)

control = Quadratic_Control.Quadratic_Control(grid_data, active_nodes ,active_ESS, conf_dict["CONTROL_DATA"])
[R,X] = control.initialize_control()
VMAX = conf_dict["CONTROL_DATA"]["VMAX"]

try:
    while True:
        active_power_dict = {}
        reactive_power_dict = {}
        active_power_ESS_dict = {}
        voltage_value = dmuObj.getDataSubset("voltage_dict")
        voltage_meas = voltage_value.get("voltage_measurements", None)

        pmu_received = dmuObj.getDataSubset("pmu_input")
        if pmu_received:
            logging.debug("pmu_input")
            logging.debug(pmu_received[0]["amplitude"])
        else:
            pass

        pv_input_value = dmuObj.getDataSubset("pv_input_dict")
        pv_input_meas = pv_input_value.get("pv_input_measurements", None)

        active_nodes = dmuObj.getDataSubset("simDict","active_nodes")
        if not active_nodes:
            active_nodes = active_nodes_old
        else:
            active_nodes = list(active_nodes)
            print(active_nodes)

        active_ESS = dmuObj.getDataSubset("simDict","ESS_nodes")
        if not active_ESS:
            active_ESS = active_ESS_old
        else:
            active_ESS = list(active_ESS)

        if voltage_value and pv_input_meas:
            ts = time.time()*1000   # time in milliseconds
            pv_nodes = list(map(lambda x: x.replace('node_',''),list(voltage_meas.keys())))
            pv_nodes = [float(i)-1 for i in pv_nodes]

            keys = ['node_'+str(int(item+1)) for item in active_nodes]
            pv_active = (dict(zip(keys, [None]*len(active_nodes)))).keys()
            num_pv = len(list(pv_active))

            ################### select input only from active nodes ###############################
            v_gen = {item:voltage_meas[item] for item in pv_active}
            pv_input = {item:pv_input_meas[item] for item in pv_active}
            v_ess = {item:voltage_meas['node_'+str(int(item)+1)] for item in active_ESS} 

            if str(os.getenv('MQTT_ENABLED')) == "true":
                v_gen = list(v_gen.values())[:-1]
                v_gen.extend([pmu_received[0]["amplitude"]/115.0])
            else:
                v_gen = list(v_gen.values())

            pv_input = list(pv_input.values())
            v_ess = list(v_ess.values())

            print("voltage node",v_gen)

            ################### re-initialize if new set of active nodes ###########################
            if active_nodes != active_nodes_old or active_ESS != active_ESS_old:
                control = Quadratic_Control.Quadratic_Control(grid_data, active_nodes, active_ESS)
                [R,X] = control.initialize_control()
                active_nodes_old = active_nodes
                active_ESS_old = active_ESS

            #########################################################################################
            ###################### Calculate the control output #####################################
            #########################################################################################
            output = control.control_(pv_input, active_power, reactive_power, R, X, active_nodes, v_gen, active_power_ESS, v_ess, VMAX)

            # update and send dictionaries
            #########################################################################################
            k = 0
            if output["DG"]["reactive_power"]:
                for key in pv_active:
                    reactive_power_dict[key] = output["DG"]["reactive_power"][k]
                    k +=1
                dmuObj.setDataSubset({"reactive_power":reactive_power_dict},"reactive_power_dict")
            k = 0
            if output["DG"]["active_power"]:
                for key in pv_active:
                    active_power_dict[key] = output["DG"]["active_power"][k]
                    k +=1
                dmuObj.setDataSubset({"active_power":active_power_dict},"active_power_dict")
            k = 0
            if output["ESS"]["active_power"]:
                for ess in active_ESS:
                    active_power_ESS_dict['node_'+str(int(ess)+1)] = output["ESS"]["active_power"][k]
                    k+=1          
                dmuObj.setDataSubset({"active_power_ESS":active_power_ESS_dict},"active_power_ESS_dict")

            print("reactive_power_dict", reactive_power_dict)

        else:
            pass

        time.sleep(0.5)

except (KeyboardInterrupt, SystemExit):
    print('simulation finished')

