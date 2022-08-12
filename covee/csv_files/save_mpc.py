import csv
import os
import numpy as np
import matplotlib.pyplot as plt


# general Plot Config
plt.rcParams["font.family"] = "serif"
plt.rcParams["figure.figsize"] = (15,7.5)
plt.rc('legend', fontsize=20, loc='upper right')    # legend fontsize

class save_mpc:

    def __init__(self, iterations, conf_dict):       
        self.lists = {"DG":{ k : [] for k in conf_dict["CONTROL_DATA"]["control_variables"]["DG"]
        },
        "ESS":{
            k : [] for k in conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]
        }}
        self.iterations = iterations

        self.lists.update({"SOC": []}  )

        self.save_path = conf_dict["SAVE_PATH"]

        cwd = os.getcwd()
        self.plot_path= os.path.join(cwd, conf_dict["PLOT_PATH"])
        self.plot_path.replace("\\", "/")
        self.conf_dict =conf_dict

    def save_list(self, output_MPC, iteration):

        if any(x =="reactive_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
            self.lists["DG"]["reactive_power"].append(output_MPC["DG"]["reactive_power"][:,1])
        if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
            self.lists["DG"]["active_power"].append(output_MPC["DG"]["active_power"][:,1])
        if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]):
            self.lists["ESS"]["active_power"].append(output_MPC["ESS"]["active_power"][:,1])
            self.lists["SOC"].append(output_MPC["ESS"]["SOC"][:,1])
        self.iterations.append(iteration)

        

  
    def save_csv(self):
        cwd = os.getcwd()
        self.wd = os.path.join(cwd, self.save_path)
        self.wd.replace("\\", "/")

        rows = zip(self.iterations,self.iterations)
        with open(os.path.join(self.wd, 'time_MPC.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
            wr = csv.writer(csv_file)
            for row in rows:
                wr.writerow(row)
        csv_file.close()

        for i in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]:
            rows = self.lists["DG"][i]
            with open(os.path.join(self.wd, i+'_DG_MPC.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()

        for i in self.conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]:
            rows = self.lists["ESS"][i]
            with open(os.path.join(self.wd, i+'_ESS_MPC.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()        

        rows = self.lists["SOC"]
        with open(os.path.join(self.wd, 'SOC_MPC.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
            wr = csv.writer(csv_file)
            for row in rows:
                wr.writerow(row)
        csv_file.close()     

    
    def Plot(self,variable, lim):
        t = {}
        x_dict = {}

        # Print
        # ============================================================
        with open(os.path.join(self.wd, str(variable)+'.csv')) as csv_file:
            var = csv.reader(csv_file, delimiter=',')
            x = list(var)
            var = np.array(x).astype("float")


        lenght = var.shape[1]-1
        for r in range(lenght):
            t.update({"t"+str(r): np.matrix(var)[:,lenght]})
            x_dict.update({"node_"+str(r): np.matrix(var)[:,r]})

        min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])

        plt.figure()
        for r in range(lenght):
            plt.plot( x_dict["node_" + str(r)][0:min_leght],linewidth=2,  c = "dimgray")
        plt.plot(x_dict["node_" + str(r-1)][0:min_leght],linewidth=2,  c = "dimgray", label=str(variable))
        plt.plot(x_dict["node_" + str(r)][0:min_leght],linewidth=2,  c = "indigo", label=str(variable)+" ["+str(r+1)+"]")
        if lim != None:
            for key in lim.keys():
                plt.plot( np.array([lim[key]]*min_leght),c = "khaki",linewidth=8, label = key)
        axes = plt.gca()
        # axes.set_ylim([0.85, 1.1])
        plt.xlabel("Iterations")
        plt.ylabel(str(variable)+"[p.u.]")
        plt.legend(facecolor='white', framealpha=1, loc='lower left')
        plt.savefig(self.plot_path+'/'+variable+'.eps')
        plt.savefig(self.plot_path+'/'+variable+'.png')
        plt.close()

