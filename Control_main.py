import numpy as np 
import csv
import os
import coloredlogs, logging, threading
from threading import Thread
from submodules.dmu.dmu import dmu
from submodules.dmu.httpSrv import httpSrv
import time
import sys
import requests
import json
import csv
import argparse
from pypower.api import *
from pypower.ext2int import ext2int
#import git

from control_strategies.Quadratic_Control_PV import Quadratic_Control_PV
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


############################ Start the Server #######################################################

''' Initialize objects '''
dmuObj = dmu()

''' Start http server '''
httpSrvThread1 = threading.Thread(name='httpSrv',target=httpSrv, args=("0.0.0.0", 8000 ,dmuObj,))
httpSrvThread1.start()

httpSrvThread2 = threading.Thread(name='httpSrv',target=httpSrv, args=("0.0.0.0", int(ext_port) ,dmuObj,))
httpSrvThread2.start()

time.sleep(2.0)
#######################################################################################################


########################################################################################################
#########################  Section for Defining the Dictionaries  ######################################
########################################################################################################

dict_ext_cntr = {
    "data_cntr" : [],
    "data_nodes" : []
}

simDict = {
    "active_nodes" : [],
    "voltage_node" : [],
    "pv_input_node" : []
}

voltage_dict = {}
active_power_dict = {}
reactive_power_dict = {}
pv_input_dict = {}

# add the simulation dictionary to mmu object
dmuObj.addElm("simDict", simDict)
dmuObj.addElm("voltage_dict", voltage_dict)
dmuObj.addElm("active_power_dict", active_power_dict)
dmuObj.addElm("reactive_power_dict", reactive_power_dict)
dmuObj.addElm("pv_input_dict", pv_input_dict)

########################################################################################################
#########################  Section for Receiving Signal  ###############################################
########################################################################################################

def voltage_input(data,  *args):
    voltage_dict = {}  
    dmuObj.setDataSubset(data,"voltage_dict")
def pv_input(data,  *args):
    pv_input_dict = {}  
    dmuObj.setDataSubset(data,"pv_input_dict")


def api_cntr_input(data,  *args):
    
    tmpData = []
    logging.debug("RECEIVED EXTERNAL CONTROL")
    logging.debug(data)       
    dmuObj.setDataSubset(data,"simDict", "active_nodes")

# Receive from external Control
dmuObj.addElm("nodes", dict_ext_cntr)
dmuObj.addRx(api_cntr_input, "nodes", "data_nodes")
# Receive voltage
dmuObj.addElm("voltage", simDict)
dmuObj.addRx(voltage_input, "voltage", "voltage_node")
# Receive pv_input
dmuObj.addElm("pv_input", simDict)
dmuObj.addRx(pv_input, "pv_input", "pv_input_node")

########################################################################################################
#########################  Section for Sending Signal  #################################################
########################################################################################################

def control_output(data, *args):

    reqData = {}
    reqData["data"] =  data
    logging.debug("##DATA##")
    # logging.debug(data)
    headers = {'content-type': 'application/json'}
    try:
        jsonData = (json.dumps(reqData)).encode("utf-8")
    except:
        logging.warn("Malformed json")
    try:
        for key in data.keys():
            if key == "active_power":
                result = requests.post("http://powerflow:8000/set/active_power/active_power_control/", data=jsonData, headers=headers)
            if key == "reactive_power":
                result = requests.post("http://powerflow:8000/set/reactive_power/reactive_power_control/", data=jsonData, headers=headers)
    except:
        logging.warn("No connection to Powerflow")

dmuObj.addRx(control_output,"active_power_dict")
dmuObj.addRx(control_output,"reactive_power_dict")

reactive_power = [0.0]*grid_data["nb"]
active_power = [0.0]*grid_data["nb"]

active_nodes = list(np.array(np.matrix(ppc["gen"])[:,0]).flatten())
active_nodes = active_nodes[1:len(active_nodes)]
pv_nodes = [float(i)-1 for i in active_nodes]
pv_nodes_old = pv_nodes    
control = Quadratic_Control_PV(grid_data, pv_nodes)
control.initialize_control()

try:
    while True:
        active_power_dict = {}
        reactive_power_dict = {}
        voltage_value = dmuObj.getDataSubset("voltage_dict")
        voltage_meas = voltage_value.get("voltage_measurements", None)
        logging.debug("voltage_measurements")
        logging.debug(voltage_meas)

        pv_input_value = dmuObj.getDataSubset("pv_input_dict")
        pv_input_meas = pv_input_value.get("pv_input_measurements", None)

        if voltage_value and pv_input_meas:
            ts = time.time()*1000   # time in milliseconds
            pv_nodes = list(map(lambda x: x.replace('node_',''),list(voltage_meas.keys())))
            num_pv = len(list(voltage_meas.values()))
            pv_nodes = [float(i)-1 for i in pv_nodes]

            v_gen = list(voltage_meas.values())#[1.07]*len(list(voltage_meas.values()))
            pv_input = list(pv_input_meas.values())
            if pv_nodes != pv_nodes_old:
                control = Quadratic_Control_PV(grid_data, pv_nodes)
                control.initialize_control()
                pv_nodes_old = pv_nodes 

            reactive_power = control.control_(pv_input, reactive_power, active_power, v_gen)

            active_power = [0.0]*num_pv
            k = 0
            for key in voltage_meas.keys():
                #updating dictionaries
                active_power_dict[key] = active_power[k]
                reactive_power_dict[key] = reactive_power[k]

                k+=1

            dmuObj.setDataSubset({"active_power":active_power_dict},"active_power_dict")
            dmuObj.setDataSubset({"reactive_power":reactive_power_dict},"reactive_power_dict")

            print("Reactive Power", reactive_power)
            print("Active Power", active_power)

        else:
            pass

        time.sleep(0.5)

except (KeyboardInterrupt, SystemExit):
    print('simulation finished')

