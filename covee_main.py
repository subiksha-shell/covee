import numpy as np 
import os
import coloredlogs, logging, threading
import time
import json
from alive_progress import alive_bar
from pypower.api import *
from pypower.ext2int import ext2int
import covee.control_strategies.utils as utils
from covee.powerflow.runPF_class import runPF_class
from covee.csv_files.save_results import save_results
from covee.csv_files.save_mpc import save_mpc
from covee.control_strategies.Scheduler import Scheduler
from covee.control_strategies.forecast.read_forecast import read_forecast
from covee.control_strategies.scheduler.save_schedule import save_schedule


'''
coloredlogs.install(level='DEBUG',
fmt='%(asctime)s %(levelname)-8s %(name)s[%(process)d] %(message)s',
field_styles=dict(
    asctime=dict(color='green'),
    hostname=dict(color='magenta'),
    levelname=dict(color='white', bold=True),
    programname=dict(color='cyan'),
    name=dict(color='blue')))
'''
logging.info("Program Start")

# Read json file and set Control Strategy and Case 
# =====================================================================================================
with open("./examples/conf.json", "r") as f:
    conf_dict = json.load(f)

module_obj = utils.select(conf_dict)
Quadratic_Control = module_obj.select_online_control()
MPC_Control = module_obj.select_MPC_control()
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


# Initialize the online control
# =====================================================================================================
control = Quadratic_Control.Quadratic_Control(grid_data, active_nodes ,active_ESS, conf_dict, conf_dict["CONTROL_DATA"],conf_dict["SCHEDULING"])
[R,X,output] = control.initialize_control()
save_obj = save_results(voltage_list = [], iterations = [], conf_dict = conf_dict)

# Initialize the powerflow
# =====================================================================================================
run_PF = runPF_class(active_nodes, active_ESS, grid_data["full_nodes"], grid_data["total_control_nodes"])
profiles = run_PF.read_profiles(conf_dict, grid_data)
active_power_dict = {}
reactive_power_dict = {}
active_power_ESS_dict = {}

# Scheduling
# =====================================================================================================
forecast = read_forecast(conf_dict)
forecast_profiles = forecast.read_csv()
scheduler_data = conf_dict["SCHEDULING"]["scheduler_data"]
scheduler_ = Scheduler(grid_data, conf_dict, forecast_profiles, scheduler_data, R, X)
output["ESS"]["SOC"] = np.array([float(scheduler_data["SOC_init"])]*len(conf_dict["CONTROL_DATA"]["active_ESS"]))
if bool(conf_dict["SCHEDULING"]["Activate"]):
    output_scheduler = scheduler_.solve()
    save_schedule = save_schedule()
    save_schedule.save_csv(output_scheduler)
    print("SCHEDULER FINISHED")   
    time.sleep(1.0)
else:
    output_scheduler = None

# Initialize the MPC
# =====================================================================================================
if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
    MPC_Control = MPC_Control.MPC_Control(grid_data,active_nodes ,active_ESS, conf_dict["CONTROL_DATA"],scheduler_data, output_scheduler, forecast_profiles, conf_dict["SCHEDULING"])
    output_MPC = MPC_Control.initialize_control()
    iter_MPC = 0
    save_mpc_obj = save_mpc(iterations = [], conf_dict = conf_dict)
else:
    iter_MPC = 0
    output_MPC = None

'''
#########################################################################################################
############################################ RUN ########################################################
#########################################################################################################
'''
with alive_bar(int(len(profiles["gen_profile"]))) as bar:  # declare your expected total
    for iter in range(int(len(profiles["gen_profile"]))):
       
        ################################# Run the PowerFlow (before the control) #####################################
        ##############################################################################################################
        [v_tot, v_gen, p, c, p_load, v_pv, v_ess] = run_PF.run_Power_Flow(ppc,output["DG"]["active_power"],output["DG"]["reactive_power"],output["ESS"]["active_power"],profiles["gen_profile"][iter],profiles["load_profile"][iter])
        
        
        ###################### Calculate the control output #####################################
        #########################################################################################
        # MPC
        # =======================
        reference = {"DG": {"active_power": {str(t+1):np.array([-0.0]*len(active_nodes)) for t in range(conf_dict["CONTROL_DATA"]["MPC_data"]["Steps"])},
                            "reactive_power": {str(t+1):np.array([-0.0]*len(active_nodes)) for t in range(conf_dict["CONTROL_DATA"]["MPC_data"]["Steps"])}
                            }}
        
        if conf_dict["POWERFLOW_DATA"]["TYPE_PROFILE"] == "fix":
            if iter%conf_dict["CONTROL_DATA"]["MPC_data"]["fix_iterations"] ==0:
                output_MPC = MPC_Control.control_(profiles, output, R, X, v_gen, v_ess, VMIN=conf_dict["CONTROL_DATA"]["VMIN"], VMAX=conf_dict["CONTROL_DATA"]["VMAX"], iter_MPC = iter_MPC, output_scheduler=output_scheduler, output_MPC=output_MPC, iter = iter, reference = reference)
                iter_MPC +=1                      
        elif conf_dict["POWERFLOW_DATA"]["TYPE_PROFILE"] == "variable" and bool(conf_dict["CONTROL_DATA"]["MPC_activate"]) and iter_MPC<(scheduler_data["Hs"]/scheduler_data["Tc"])-conf_dict["CONTROL_DATA"]["MPC_data"]["Steps"]:
            if iter%(len(profiles["gen_profile"])/(scheduler_data["Hs"]/scheduler_data["Tc"]))==0.0:
                output_MPC = MPC_Control.control_(profiles, output, R, X, v_gen, v_ess, VMIN=conf_dict["CONTROL_DATA"]["VMIN"], VMAX=conf_dict["CONTROL_DATA"]["VMAX"], iter_MPC = iter_MPC, output_scheduler=output_scheduler, output_MPC=output_MPC, iter=iter, reference = reference)
                iter_MPC +=1
        else:
            pass

        # Online
        # =======================        
        output = control.control_(profiles, output, R, X, v_gen, v_ess, VMIN=conf_dict["CONTROL_DATA"]["VMIN"], VMAX=conf_dict["CONTROL_DATA"]["VMAX"], iter=iter, iter_MPC = iter_MPC, output_MPC = output_MPC)

        # ################################# Run the PowerFlow (after the control)#####################################
        #############################################################################################################
        [v_tot,v_gen,p,c,p_load,v_pv,v_ess] = run_PF.run_Power_Flow(ppc,output["DG"]["active_power"],output["DG"]["reactive_power"],output["ESS"]["active_power"],profiles["gen_profile"][iter],profiles["load_profile"][iter])


        # update the dictionaries
        #########################################################################################
        [reactive_power_dict, active_power_dict, active_power_ESS_dict] = additional.update_dict(output, reactive_power_dict, active_power_dict, active_power_ESS_dict)      
        save_obj.save_list(output, v_tot, iter)
        if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
            save_mpc_obj.save_list(output_MPC,iter)

        
        bar()

    # Save the data in csv files
    # =====================================================================================================
    save_obj.save_csv()
    if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
        save_mpc_obj.save_csv()
    # Plot function
    # =====================================================================================================
    save_obj.Plot("voltage", {"VMAX" : conf_dict["CONTROL_DATA"]["VMAX"],
                              "VMIN" : conf_dict["CONTROL_DATA"]["VMIN"]})
    for i in conf_dict["CONTROL_DATA"]["control_variables"]["DG"]:
        save_obj.Plot(i+'_DG', None)
        if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
            save_mpc_obj.Plot(i+'_DG_MPC', None)
    for i in conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]:
        save_obj.Plot(i+"_ESS", None)
        if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
            save_mpc_obj.Plot(i+'_ESS_MPC', None)
        save_obj.Plot("SOC", None)
        if bool(conf_dict["CONTROL_DATA"]["MPC_activate"]):
            save_mpc_obj.Plot('SOC_MPC', None)

    logging.info('simulation finished')


          