import numpy as np 
import csv
import os
import coloredlogs, logging, threading
from threading import Thread
from submodules.dmu.dmu import dmu
from submodules.dmu.httpSrv import httpSrv
from submodules.dmu.mqttClient import mqttClient
import time
import sys
import requests
import json
import csv
import argparse
from pypower.api import *
from pypower.ext2int import ext2int
import random

from control_strategies.Quadratic_Control_Centralized_DualAscent import Quadratic_Control as Control

from cases.LV_SOGNO import LV_SOGNO as case


parser = argparse.ArgumentParser()
parser.add_argument('--ext_port', nargs='*', required=True)
args = vars(parser.parse_args())
ext_port = args['ext_port'][0]


def system_info(ppc):
    # Gather information about the system
    # =============================================================
    baseMVA, bus, gen, branch, cost, VMAX, VMIN = \
        ppc["baseMVA"], ppc["bus"], ppc["gen"], ppc["branch"], ppc["gencost"], ppc["VMAX"], ppc["VMIN"]

    nb = bus.shape[0]                        # number of buses
    ng = gen.shape[0]                        # number of generators
    nbr = branch.shape[0]                    # number of branches

    for i in range(int(nb)):
        if bus[i][BUS_TYPE] == 3.0:
            pcc = i
        else:
            pass

    grid_data = {"baseMVA":baseMVA,"branch":branch,"pcc":pcc,"bus":bus,"gen":gen,"nb":nb,"ng":ng,"nbr":nbr}

    return grid_data

ppc = case()
ppc = ext2int(ppc)      # convert to continuous indexing starting from 0
BUS_TYPE = 1
grid_data = system_info(ppc)

coloredlogs.install(level='DEBUG',
fmt='%(asctime)s %(levelname)-8s %(name)s[%(process)d] %(message)s',
field_styles=dict(
    asctime=dict(color='green'),
    hostname=dict(color='magenta'),
    levelname=dict(color='white', bold=True),
    programname=dict(color='cyan'),
    name=dict(color='blue')))
logging.info("Program Start")

external_mqtt = "no"

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

############################ Start the Server #######################################################

''' Initialize objects '''
dmuObj = dmu()

''' Start mqtt client '''
mqttObj = mqttClient(mqtt_url, dmuObj, mqtt_port, mqtt_user, mqtt_password)

''' Start http server '''
httpSrvThread2 = threading.Thread(name='httpSrv',target=httpSrv, args=("0.0.0.0", int(ext_port) ,dmuObj,))
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

# add the simulation dictionary to mmu object
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


active_nodes = list(np.array(np.matrix(ppc["gen"])[:,0]).flatten())
active_nodes = [3,4,5,6,8,9,10,12,13,14,15,16,17,18,19]#active_nodes[1:len(active_nodes)]
active_nodes_old = active_nodes    
reactive_power = [0.0]*len(active_nodes)
active_power = [0.0]*len(active_nodes)
voltage_value_old = [0.0]*len(active_nodes)
active_ESS = active_nodes
active_ESS_old = active_ESS
active_power_ESS = [0.0]*len(active_ESS)
print("active_nodes", active_nodes)
control = Control(grid_data, active_nodes,active_ESS)
[R,X] = control.initialize_control()

try:
    while True:
        active_power_dict = {}
        reactive_power_dict = {}
        active_power_ESS_dict = {}
        voltage_value = dmuObj.getDataSubset("voltage_dict")
        voltage_meas = voltage_value.get("voltage_measurements", None)
        # logging.debug("voltage_measurements")
        # logging.debug(voltage_meas)

        pmu_received = dmuObj.getDataSubset("pmu_input")
        if pmu_received:
            logging.debug("pmu_input")
            logging.debug(pmu_received[0]["amplitude"])
        else:
            pass

        pv_input_value = dmuObj.getDataSubset("pv_input_dict")
        pv_input_meas = pv_input_value.get("pv_input_measurements", None)
        # logging.debug("pv_input_measurements")
        # logging.debug(pv_input_meas)

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

            # print("active_nodes", active_nodes)
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
                control = Control(grid_data, active_nodes)
                [R,X] = control.initialize_control()
                active_nodes_old = active_nodes
                active_ESS_old = active_ESS

            # if v_gen != voltage_value_old:
            [active_power,reactive_power,active_power_ESS] = control.control_(pv_input, active_power, reactive_power, R, X, active_nodes, v_gen, active_power_ESS, v_ess)
            active_power_ESS = [0.0]*len(active_ESS)
            voltage_value_old = v_gen
            # else:
            #     pass

            # updating dictionaries
            k = 0
            for key in pv_active:
                active_power_dict[key] = active_power[k]
                reactive_power_dict[key] = reactive_power[k]
                k +=1
            k = 0
            for ess in active_ESS:
                active_power_ESS_dict['node_'+str(int(ess)+1)] = active_power_ESS[k]
                k+=1
 
            dmuObj.setDataSubset({"active_power":active_power_dict},"active_power_dict")
            dmuObj.setDataSubset({"reactive_power":reactive_power_dict},"reactive_power_dict")
            dmuObj.setDataSubset({"active_power_ESS":active_power_ESS_dict},"active_power_ESS_dict")

            print("Reactive Power", reactive_power_dict)
            # print("Active Power", active_power_dict)
            # print("Active Power ESS", active_power_ESS_dict)

        else:
            pass

        time.sleep(0.5)

except (KeyboardInterrupt, SystemExit):
    print('simulation finished')

