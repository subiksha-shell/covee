import csv
import os
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "serif"
plt.rcParams["figure.figsize"] = (15,7.5)
plt.rcParams.update({'font.size': 26})
plt.rc('legend', fontsize=20, loc='upper right')    # legend fontsize

class read_forecast:

    def __init__(self,conf_dict):

        self.pvproduction = []
        self.demandprofile_P = []
        self.pvmeasurements = []
        self.demandmeasurements = []
        self.conf_dict = conf_dict
        self.profiles_out = {}


    def read_csv(self):


        profiles = {
            'PV_forecast' : [self.conf_dict["SCHEDULING"]["scheduler_data"]["PROFILE"]["GEN_PROFILE"],self.pvmeasurements],
            'Load_forecast' : [self.conf_dict["SCHEDULING"]["scheduler_data"]["PROFILE"]["LOAD_PROFILE"],self.demandmeasurements],
        }

        # Read CSV FILES
        # =============================================================

        cwd = os.getcwd()
        wd = os.path.join(cwd, 'covee/control_strategies/forecast')
        wd.replace("\\", "/")    
        k = 1
        for key in profiles:

            with open(os.path.join(wd, profiles[key][0])) as csv_file: 
                pvproduction = csv.reader(csv_file, delimiter=',')
                x = list(pvproduction)
                profiles[key][1] = np.array(x).astype("float")
            
            name = profiles[key][0].replace('.csv','')
            list_plot = profiles[key][1] 
            for j in range(profiles[key][1].shape[1]):
                globals()[name+'_' + str(j)] = []
                for i in range(int(list_plot.shape[0]) - 1):
                    globals()[name+'_' + str(j)].append(list_plot[i][j])

            # plt.figure(k)
            # for j in range(list_plot.shape[1]):
            #     plt.plot(globals()[name+'_' + str(j)],'grey')
            # plt.plot(globals()[name+'_' + str(j)],'deepskyblue', label='bus'+str(j))
            # plt.plot(globals()[name+'_' + str(j-1)],'chartreuse', label='bus'+str(j-1))

            # axes = plt.gca()
            # plt.xlabel("Time [h]")
            # plt.ylabel(name+'[p.u.]')
            # plt.legend()
            # plt.savefig(wd1+'/plots/'+name+'.eps')
            # plt.savefig(wd1+'/plots/'+name+'.png')
            # k+=1
        
        self.profile_out = {"gen_forecast" : np.array(profiles['PV_forecast'][1]),"load_forecast": np.array(profiles['Load_forecast'][1])}
            


        return self.profile_out