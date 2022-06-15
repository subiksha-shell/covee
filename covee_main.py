import numpy as np 
import os
import coloredlogs, logging, threading
import time
import sys
import json
from pypower.api import *
from pypower.ext2int import ext2int
import control_strategies.utils as utils
from importlib import import_module
from powerflow.powerflow_class.runPF_class import runPF_class
from csv_files.save_results import save_results

coloredlogs.install(level='DEBUG',
fmt='%(asctime)s %(levelname)-8s %(name)s[%(process)d] %(message)s',
field_styles=dict(
    asctime=dict(color='green'),
    hostname=dict(color='magenta'),
    levelname=dict(color='white', bold=True),
    programname=dict(color='cyan'),
    name=dict(color='blue')))

'''
'''
logging.info("Program Start")

# Read json file and set Control Strategy and Case 
# =====================================================================================================
with open("./conf.json", "r") as f:
    conf_dict = json.load(f)

module_obj = utils.select(conf_dict)
Quadratic_Control = module_obj.select_control()
case = module_obj.select_case()
# =====================================================================================================

'''
#########################################################################################################
##################################### INITIALIZE ########################################################
#########################################################################################################
'''
# Get the grid data infos
# =====================================================================================================
additional = utils.additional() 

conf_dict = additional.convert_index(conf_dict)
active_nodes = conf_dict["CONTROL_DATA"]["active_nodes"]   # number of active DGs
active_ESS = conf_dict["CONTROL_DATA"]["active_ESS"]       # number of active ESSs
logging.info("active_nodes"+str(active_nodes))

ppc_obj = case.case_()
ppc = ppc_obj.case()
ppc = ext2int(ppc)      # convert to continuous indexing starting from 0
BUS_TYPE = 1
[grid_data,reactive_power,active_power,active_power_ESS] = additional.system_info(ppc,BUS_TYPE, active_nodes, active_ESS)


# Initialize the control
# =====================================================================================================
control = Quadratic_Control.Quadratic_Control(grid_data, active_nodes ,active_ESS, conf_dict["CONTROL_DATA"])
[R,X,output] = control.initialize_control()
VMAX = conf_dict["CONTROL_DATA"]["VMAX"]
save_obj = save_results(voltage_list = [], iterations = [], conf_dict = conf_dict)

# Initialize the powerflow
# =====================================================================================================
run_PF = runPF_class(active_nodes, active_ESS, grid_data["full_nodes"], grid_data["total_control_nodes"])
profiles = run_PF.read_profiles(conf_dict, grid_data)
active_power_dict = {}
reactive_power_dict = {}
active_power_ESS_dict = {}

'''
#########################################################################################################
############################################ RUN ########################################################
#########################################################################################################
'''
for iter in range(int(len(profiles["gen_profile"]))):

    ################################# Run the PowerFlow #####################################
    #########################################################################################
    [v_tot,v_gen,p,c,p_load,v_pv,v_ess] = run_PF.run_Power_Flow(ppc,output["DG"]["active_power"],output["DG"]["reactive_power"],output["ESS"]["active_power"],profiles["gen_profile"][iter],profiles["load_profile"][iter])

    ###################### Calculate the control output #####################################
    #########################################################################################
    output = control.control_(profiles["gen_profile"][iter][active_nodes], output, R, X, active_nodes, v_gen, v_ess, VMAX)

    # update the dictionaries
    #########################################################################################
    [reactive_power_dict, active_power_dict, active_power_ESS_dict] = additional.update_dict(output, reactive_power_dict, active_power_dict, active_power_ESS_dict)      
    save_obj.save_list(output, v_tot, iter)

    print("voltage ", v_tot)
    print("pv_input ", profiles["gen_profile"][iter][active_nodes])
    print("reactive_power_dict ", reactive_power_dict)

# Save the data in csv files
# =====================================================================================================
save_obj.save_csv()
# Plot function
# =====================================================================================================
save_obj.Plot("voltage")
for i in conf_dict["CONTROL_DATA"]["control_variables"]["DG"]:
    save_obj.Plot(i+'_DG')
for i in conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]:
    save_obj.Plot(i+"_ESS")

logging.info('simulation finished')