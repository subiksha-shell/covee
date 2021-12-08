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
from csv_files.read_profiles import read_profiles
from control_strategies.model_predictive_control.matrix_calc import matrix_calc


from control_strategies.MPC_Control_Centralized_OSQP import MPC_Control as Control

from cases.case_10_nodes import case_10_nodes as case


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

if bool(os.getenv('MQTT_ENABLED')):
    mqtt_url = "mqtt"#str(os.getenv('MQTTURL'))
    mqtt_port = 1883#int(os.getenv('MQTTPORT'))
    mqtt_user = ""#str(os.getenv('MQTTUSER'))
    mqtt_password = ""#str(os.getenv('MQTTPASS'))
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
    "ESS_nodes" : [],
    "test_value" : None
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
dmuObj.addElm("flex_request_dict", {})

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
dmuObj.addElm("test", {})
dmuObj.addElmMonitor(api_cntr_input, "nodes", "data_nodes")
dmuObj.addElmMonitor(api_cntr_input, "test", "data_test")

# Receive voltage
dmuObj.addElm("voltage", simDict)
mqttObj.attachSubscriber("/voltage_control/measuremnts/voltage","json","voltage_dict")
# Receive pv_input
dmuObj.addElm("pv_input", simDict)
mqttObj.attachSubscriber("/voltage_control/measuremnts/pv","json","pv_input_dict")
# Receive pmu_meas
dmuObj.addElm("pmu_meas", {})
mqttObj.attachSubscriber("/edgeflex/edgepmu0/ch0/amplitude","json","pmu_input")
# Receive flexibility_input
dmuObj.addElm("flex_input", {})
mqttObj.attachSubscriber("/edgeflex/goflex/flex_result","json","flex_input")
########################################################################################################
#########################  Section for Sending Signal  #################################################
########################################################################################################

mqttObj.attachPublisher("/voltage_control/control/active_power","json","active_power_dict")
mqttObj.attachPublisher("/voltage_control/control/reactive_power","json","reactive_power_dict")
mqttObj.attachPublisher("/voltage_control/control/active_power_ESS","json","active_power_ESS_dict")
mqttObj.attachPublisher("/edgeflex/goflex/flex_request","json","flex_request_dict")

resize = "no"
flexibility = "yes"
predictions = 1
activate_SoC = 0.0

'''
#################### Initialize ################################################
'''
controllable_variables = ["active_power", "reactive_power"]

active_nodes =[3,4,5,6,8,9]
if resize == "yes":
    full_nodes = list(np.array(np.matrix(ppc["gen"])[:,0]).flatten())
    modify_obj_function = {r: np.array([1.0]*len(full_nodes)) for r in controllable_variables}
else:
    full_nodes = active_nodes
    modify_obj_function = {r: np.array([1.0]*len(full_nodes)) for r in controllable_variables}
active_nodes_old = active_nodes  
active_ESS = active_nodes
active_ESS_old = active_ESS

reactive_power =  {"pred_"+str(s+1): [0.0]*len(active_nodes)  for s in range(predictions)}
active_power = {"pred_"+str(s+1): [0.0]*len(active_nodes) for s in range(predictions)}
active_power_ess = {"pred_"+str(s+1): [0.0]*len(active_ESS) for s in range(predictions)}

reference = {"active_power": {"pred_"+str(s+1):  [0.0]*len(active_nodes) for s in range(predictions)},
                "reactive_power": {"pred_"+str(s+1): [0.0]*len(active_nodes) for s in range(predictions)},
                "active_power_ess": {"pred_"+str(s+1): [0.0]*len(active_ESS) for s in range(predictions)},
                "voltage": {"pred_"+str(s+1): [1.03]*len(active_nodes) for s in range(predictions)}
                }
pv_production = {"pred_"+str(s+1): [0.0]*len(active_nodes) for s in range(predictions)}
print("active_nodes", active_nodes)

'''
#######################################################################################
'''


'''
#################### Calculate Matrixes ################################################
'''
control = Control(grid_data, active_nodes, active_ESS,full_nodes)
control.initialize_control()
if resize == "yes":
    calculate_matrix = matrix_calc(grid_data,full_nodes[1:])
else:
    calculate_matrix = matrix_calc(grid_data,active_nodes)
[R,X] = calculate_matrix.calculate()

if resize == "yes":
    calculate_matrix = matrix_calc(grid_data,full_nodes[1:])
else:
    calculate_matrix = matrix_calc(grid_data,active_ESS)
[R_ess,X_ess] = calculate_matrix.calculate()

n_ess = np.shape(R_ess)[0]
SoC_value = {"pred_"+str(s+1): np.array([51.0]*len(active_ESS))  for s in range(predictions)}

'''
#######################################################################################
'''

# read profiles from CSV files
# =======================================================================
profiles = read_profiles()
PV_list = profiles.read_csv()

k_pred = 1

try:
    while True:

        active_power_dict = {}
        reactive_power_dict = {}
        active_power_ESS_dict = {}
        voltage_value = dmuObj.getDataSubset("voltage_dict")
        voltage_meas = voltage_value.get("voltage_measurements", None)
        logging.debug("voltage_measurements")
        logging.debug(voltage_meas)

        pmu_received = dmuObj.getDataSubset("pmu_input")
        # logging.debug("pmu_input")
        # logging.debug(pmu_received)

        pv_input_value = dmuObj.getDataSubset("pv_input_dict")
        pv_input_meas = pv_input_value.get("pv_input_measurements", None)
        logging.debug("pv_input_measurements")
        logging.debug(pv_input_meas)

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

            print("pv_input_meas", pv_input_meas)
            print("voltage_meas", voltage_meas)
            
            keys = ['node_'+str(int(item+1)) for item in active_nodes]
            pv_active = (dict(zip(keys, [None]*len(active_nodes)))).keys()
            num_pv = len(list(pv_active))
            print("pv_active", pv_active)

            ################### select input only from active nodes ###############################
            v_gen = {item:voltage_meas[item] for item in pv_active}
            pv_input = {item:pv_input_meas[item] for item in pv_active}
            v_ess = {item:voltage_meas['node_'+str(int(item)+1)] for item in active_ESS} 

            v_gen = list(v_gen.values())
            pv_input = list(pv_input.values())
            v_ess = list(v_ess.values())

            ################### re-initialize if new set of active nodes ###########################
            if active_nodes != active_nodes_old or active_ESS != active_ESS_old:
                control = Control(grid_data, active_nodes, active_nodes_ESS,full_nodes)
                control.initialize_control()
                calculate_matrix = matrix_calc(grid_data,full_nodes[1:])
                [R,X] = calculate_matrix.calculate()

            for s in range(predictions):
                if s == 0:
                    pv_production["pred_"+str(s+1)] = pv_input
                else:
                    pv_production["pred_"+str(s+1)] = pv_input #to modify with the forecast
            forecast = pv_production

            if resize == "yes":
                v_tot = v_tot[1:]
            else:
                v_tot = v_gen
            VMAX = [1.048]*len(v_tot)
            if activate_SoC == 1.0:
                SoC_ref = {"pred_"+str(s+1): np.array([SoC_steps[iteration+s]]*n_ess) for s in range(predictions)}
            else:
                SoC_ref = {"pred_"+str(s+1): np.array([0.0]*n_ess) for s in range(predictions)}
            
            activation_nodes = [1]*len(active_nodes)      

            print("REFERENCE", reference)   
            logging.debug("v_tot " +str(v_tot))

            [active_power,reactive_power, active_power_ess, SoC_value, v_tot_dict] = control.control_(resize, forecast, pv_production, active_power, reactive_power,active_power_ess,R, X, R_ess, 
                                                            active_nodes, v_tot, VMAX, reference,activation_nodes, predictions, controllable_variables, k_pred, SoC_value, SoC_ref, activate_SoC, modify_obj_function)
           
            active_power_ESS = {"pred_"+str(s+1): [0.0]*len(active_ESS) for s in range(predictions)}

            time.sleep(0.5)
            k_pred +=1
            
            # updating dictionaries          
            for s in range(predictions):
                k = 0
                active_power_dict.update({"pred_"+str(s+1): {}})
                reactive_power_dict.update({"pred_"+str(s+1): {}})
                active_power_ESS_dict.update({"pred_"+str(s+1): {}})
                for key in pv_active:
                    active_power_dict["pred_"+str(s+1)][key] = active_power["pred_"+str(s+1)][k]
                    reactive_power_dict["pred_"+str(s+1)][key] = reactive_power["pred_"+str(s+1)][k]
                    k +=1
                k = 0
                for ess in active_ESS:
                    active_power_ESS_dict['node_'+str(int(ess)+1)] = {"pred_"+str(s+1): active_power_ESS["pred_"+str(s+1)][k] for s in range(predictions)}
                    k+=1
 
            if flexibility == "yes":
                # send the flex request
                flex_request = {"active_power": active_power_dict, "reactive_power": reactive_power_dict, "active_power_ESS": active_power_ESS_dict}
                print(flex_request)
                dmuObj.setDataSubset({"variables":controllable_variables,"flex_dict":flex_request},"flex_request_dict")
                flex_response = dmuObj.getDataSubset("flex_input")
                print("flex_response_bool", bool(flex_response))
                if bool(flex_response) is False:
                    logging.info("no flexibility response")
                    time.sleep(0.3)
                    ### re -initialize  #########################
                    reactive_power =  {"pred_"+str(s+1): [0.0]*len(active_nodes)  for s in range(predictions)}
                    active_power = {"pred_"+str(s+1): [0.0]*len(active_nodes) for s in range(predictions)}
                    active_power_ess = {"pred_"+str(s+1): [0.0]*len(active_ESS) for s in range(predictions)}
                    v_tot = [0.0]*len(v_tot)
                    dmuObj.setDataSubset({"active_power":{node: 0.0 for node in pv_active}},"active_power_dict")
                    dmuObj.setDataSubset({"reactive_power":{node: 0.0 for node in pv_active}},"reactive_power_dict")
                    dmuObj.setDataSubset({"active_power_ESS":{node: 0.0 for node in pv_active}},"active_power_ESS_dict")
                else:
                    flex_input = dmuObj.getDataSubset("flex_input")
                    logging.info("flex input " + str(flex_input))
                    modify_obj_function = {s: np.ones((len(active_nodes))) for s in controllable_variables}
                    for ref_var in flex_input["variable"]:
                        modify_obj_function[ref_var][flex_input[ref_var]["vector_position"]] = 1e5
                        ref_input = np.zeros((len(active_nodes)))
                        r = 0
                        logging.info(flex_input[ref_var]["vector_position"][r])
                        for k in range(len(active_nodes)):
                            if k == flex_input[ref_var]["vector_position"][r]:
                                ref_input[k] = flex_input[ref_var]["flex_value"][r]
                                print(r+1)
                                if (r+1) < len(flex_input[ref_var]["vector_position"]):
                                    r += 1
                                else:
                                    pass
                            else:
                                pass                          
                        reference[ref_var]["pred_"+str(flex_input[ref_var]["prediction"])] = (ref_input).tolist()
                        print(reference[ref_var])
                        print(modify_obj_function)


                    [active_power,reactive_power, active_power_ess, SoC_value, v_tot_dict] = control.control_(resize, forecast, pv_production, active_power, reactive_power,active_power_ess,R, X, R_ess, 
                                                                    active_nodes, v_tot, VMAX, reference,activation_nodes, predictions, controllable_variables, k_pred, SoC_value, SoC_ref, activate_SoC, modify_obj_function)
                
                    active_power_ESS = {"pred_"+str(s+1): [0.0]*len(active_ESS) for s in range(predictions)}                    

                    # updating dictionaries          
                    for s in range(predictions):
                        k = 0
                        active_power_dict.update({"pred_"+str(s+1): {}})
                        reactive_power_dict.update({"pred_"+str(s+1): {}})
                        active_power_ESS_dict.update({"pred_"+str(s+1): {}})
                        for key in pv_active:
                            active_power_dict["pred_"+str(s+1)][key] = active_power["pred_"+str(s+1)][k]
                            reactive_power_dict["pred_"+str(s+1)][key] = reactive_power["pred_"+str(s+1)][k]
                            k +=1
                        k = 0
                        for ess in active_ESS:
                            active_power_ESS_dict['node_'+str(int(ess)+1)] = {"pred_"+str(s+1): active_power_ESS["pred_"+str(s+1)][k] for s in range(predictions)}
                            k+=1

                    for multiple in range(5):
                        dmuObj.setDataSubset({"active_power":active_power_dict["pred_1"]},"active_power_dict")
                        dmuObj.setDataSubset({"reactive_power":reactive_power_dict["pred_1"]},"reactive_power_dict")
                        dmuObj.setDataSubset({"active_power_ESS":active_power_ESS_dict["pred_1"]},"active_power_ESS_dict")

                        print("active Power", active_power_dict["pred_1"])
                    break
            else:
                dmuObj.setDataSubset({"active_power":active_power_dict["pred_1"]},"active_power_dict")
                dmuObj.setDataSubset({"reactive_power":reactive_power_dict["pred_1"]},"reactive_power_dict")
                dmuObj.setDataSubset({"active_power_ESS":active_power_ESS_dict["pred_1"]},"active_power_ESS_dict")
            

            print("Reactive Power", reactive_power_dict["pred_1"])
            print("Active Power", active_power_dict["pred_1"])
            print("Active Power ESS", active_power_ESS_dict["pred_1"])

        else:
            k_pred=0

        time.sleep(2)

except (KeyboardInterrupt, SystemExit):
    print('simulation finished')

