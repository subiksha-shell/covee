
import cvxpy as cp
import numpy as np
import time
from .scheduler.optimization_data import optimization_data
from .scheduler.matrix_calc import matrix_calc
import coloredlogs, logging, threading


class Scheduler():

    def __init__(self, grid_data, conf_dict,forecast, scheduler_data, R, X):
        self.grid_data = grid_data
        self.forecast = forecast
        self.P_G = forecast['gen_forecast']
        self.P_D = forecast['load_forecast']
        self.conf_dict = conf_dict
        self.active_nodes = conf_dict["CONTROL_DATA"]["active_nodes"]   # number of active DGs
        self.active_ESS = conf_dict["CONTROL_DATA"]["active_ESS"]       # number of active ESSs

        scheduler_data.update({"Steps" : int(scheduler_data["Hs"]/scheduler_data["Tc"]) }) # Number of steps for the objective function
        scheduler_data.update({"P_DG_iMAX" : [max(forecast['gen_forecast'][i,:]) for i in range(scheduler_data["Steps"])] })   # For the calculation of the weights according to the range of power
        scheduler_data.update({"P_DG_MAX" : 1.0*np.array([forecast['gen_forecast'][i,:] for i in range(scheduler_data["Steps"])]), "P_DG_MIN" : 0.0*np.array([forecast['gen_forecast'][i,:] for i in range(scheduler_data["Steps"])])})   # P DG Power Limits
        scheduler_data.update({"Q_DG_MAX" : 0.4*scheduler_data["P_DG_MAX"] , "Q_DG_MIN" : -0.4*scheduler_data["P_DG_MAX"]})   # Q DG Power Limits
        scheduler_data.update({"P_ESS_MAX" :  np.array(scheduler_data["P_ESS_MAX"]*len(self.active_ESS)), "P_ESS_MIN" : np.array(scheduler_data["P_ESS_MIN"]*len(self.active_ESS))})   # P ESS Power Limits
        scheduler_data.update({"load_forecast" : np.array([forecast['load_forecast'][i,:] for i in range(scheduler_data["Steps"])])})

        self.scheduler_data = scheduler_data

        self.matrixes = {}
        self.matrixes.update({
            "R_tot" : R, "X_tot" : X,
            "n_DG" : len(self.active_nodes),
            "n_ESS" : len(self.active_ESS),
            "n_tot" : len(self.grid_data["full_nodes"]),
            "active_DG_nodes" : [int(i) for i in self.active_nodes],
            "active_ESS_nodes" : [int(i) for i in self.active_ESS],
            "tot_nodes" :  [int(i) for i in self.active_nodes] #for now we use active DGs as total nodes    [int(i) for i in self.grid_data["full_nodes"]]
        })

    def solve(self):
        T = self.scheduler_data["Steps"]
        n_tot = self.matrixes["n_tot"]
        n_DG = self.matrixes["n_DG"]
        n_ESS = self.matrixes["n_ESS"]

        M = self.scheduler_data["M"]

        self.active_nodes = self.matrixes["active_DG_nodes"]
        self.active_nodes_ESS = self.matrixes["active_ESS_nodes"]

        # recalculate the matrixes considering the DGs and ESSs together
        diff__ = list(set(self.active_nodes_ESS)- set(self.active_nodes))
        self.active_tot = np.sort(self.active_nodes+diff__)
        self.n = len(self.active_tot)
        n_tot = self.n
        calculate_matrix_full = matrix_calc(self.grid_data, self.active_tot) 
        [R,X] = calculate_matrix_full.calculate()

        self.scheduler_data.update({"P_ESS_MAX" :  np.array(self.scheduler_data["P_ESS_MAX"]*self.n), "P_ESS_MIN" : np.array(self.scheduler_data["P_ESS_MIN"]*self.n)})   # P ESS Power Limits
        self.scheduler_data.update({"V_slack": np.array([self.scheduler_data["V_slack"]]*self.n) })
        v_ref = np.array([self.scheduler_data["v_ref"]]*n_tot)
        
        '''
        ####################################### Create the weighted matrixes and Define the control variables ########################################
        '''
        weights = {} 
        if self.scheduler_data["control_variables"]["DG"]:
            factor_DG = 1e6
            factor_DG_array = np.ones((self.n))
            if any(i == "reactive_power" for i in self.scheduler_data["control_variables"]["DG"]):
                weights.update({"Q_DG": [self.scheduler_data["Weights_CVXPY"]["DG"]["reactive_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["Q_DG"][k] = factor_DG*weights["Q_DG"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        factor_DG_array[k] = factor_DG*factor_DG_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                q_DG = cp.Variable((n_tot,T))
                W_Q_DG = np.diag(weights["Q_DG"])
            else:
                q_DG = None

            if any(i == "active_power" for i in self.scheduler_data["control_variables"]["DG"]):
                weights.update({"P_DG": [self.scheduler_data["Weights_CVXPY"]["DG"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["P_DG"][i] = factor_DG*weights["P_DG"][i] # To esclude nodes where no DGs are instualled (in the matrix weights)
                        factor_DG_array[i] = factor_DG*factor_DG_array[i] # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                p_DG = cp.Variable((n_tot,T))
                W_P_DG = np.diag(weights["P_DG"])
            else:
                p_DG = None

        # Create variables and matrixes ESSs
        if self.scheduler_data["control_variables"]["ESS"]:
            factor_ESS_array = np.ones((self.n))
            factor_ESS = 1e6
            if any(i == "active_power" for i in self.scheduler_data["control_variables"]["ESS"]):
                weights.update({"P_ESS": [self.scheduler_data["Weights_CVXPY"]["ESS"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes_ESS):
                        pass
                    else:


                        weights["P_ESS"][i] = factor_ESS*weights["P_ESS"][i] # To esclude nodes where no ESSs are instualled (in the matrix weights)
                        factor_ESS_array[i] = factor_ESS*factor_ESS_array[i] # To esclude nodes where no ESSs are instualled (later in the constraints)
                    k+=1
                p_ESS = cp.Variable((n_tot,T))
                SOC = cp.Variable((n_tot,T)) 
                W_P_ESS = np.diag(weights["P_ESS"])
            else:
                p_ESS = None

        # define voltage variable and relaxation variables 
        v = cp.Variable((n_tot,T))
        rho_vmax = cp.Variable((n_tot,T))
        rho_vmin = cp.Variable((n_tot,T))
        W_V = np.diag([self.scheduler_data["Weights_CVXPY"]["V"]]* self.n)

        # Trasform Matrixes in arrays
        R_tot = np.array(R)
        X_tot = np.array(X)

        # Calculate inital voltage
        v_0 = self.scheduler_data["V_slack"] + R_tot@self.scheduler_data["P_DG_MAX"][0][self.active_tot] \
                                        - R_tot@self.scheduler_data["load_forecast"][0][self.active_tot]
        # Calculate uncontrolled voltage
        v_unctr = np.zeros((n_tot,T+1))
        for t in range(T):
            v_unctr[:,t] = self.scheduler_data["V_slack"] + R_tot@self.scheduler_data["P_DG_MAX"][t][self.active_tot] \
                                        - R_tot@self.scheduler_data["load_forecast"][t][self.active_tot]
            

        cost = 0
        constr = []
              
        for t in range(T-1):
            if self.scheduler_data["control_variables"]["DG"]:
                # Calculate the DG reactive power constraints and costs         
                if q_DG:
                    constr += [ cp.multiply(factor_DG_array,q_DG[:,t+1]) <= self.scheduler_data["Q_DG_MAX"][t+1][self.active_tot], 
                                -cp.multiply(factor_DG_array,q_DG[:,t+1]) <= -self.scheduler_data["Q_DG_MIN"][t+1][self.active_tot],
                            ]
                    cost += (1/2)*cp.quad_form(q_DG[:,t+1], W_Q_DG)
                    # contribution to voltage regulation
                    add_Q_DG = X_tot@q_DG[:,t+1]                
                else:
                    add_Q_DG = np.array([0.0]*n_tot) 

                # Calculate the DG active power constraints and costs 
                if p_DG:
                    constr += [ cp.multiply(factor_DG_array,p_DG[:,t+1]) <= self.scheduler_data["P_DG_MAX"][t+1][self.active_tot], 
                                -cp.multiply(factor_DG_array,p_DG[:,t+1]) <= -self.scheduler_data["P_DG_MIN"][t+1][self.active_tot],
                            ]
                    cost += (1/2)*cp.quad_form(p_DG[:,t+1], W_P_DG)
                    # contribution to voltage regulation
                    add_P_DG = R_tot@p_DG[:,t+1]               
                else:
                    add_P_DG = np.array([0.0]*n_tot) #

            else:
                add_Q_DG = np.array([0.0]*n_tot) 
                add_P_DG = np.array([0.0]*n_tot) 

            if self.scheduler_data["control_variables"]["ESS"]:
                # Calculate the ESS active power constraints and costs 
                if p_ESS:
                    constr += [ cp.multiply(factor_ESS_array,p_ESS[:,t+1]) <= self.scheduler_data["P_ESS_MAX"],
                                -cp.multiply(factor_ESS_array,p_ESS[:,t+1]) <= -self.scheduler_data["P_ESS_MIN"],
                                SOC[:,t+1] <= np.array(self.scheduler_data["SOC_MAX"]),
                                -SOC[:,t+1] <= -np.array(self.scheduler_data["SOC_MIN"])
                            ]
                    constr += [SOC[:,t+1] == SOC[:,t] - 0.2*p_ESS[:,t+1] ]

                    cost += (1/2)*cp.quad_form(p_ESS[:,t+1], W_P_ESS)
                    # contribution to voltage regulation
                    add_P_ESS = R_tot@p_ESS[:,t+1]               
                else:
                    add_P_ESS = np.array([0.0]*n_tot) 
            else:
                add_P_ESS = np.array([0.0]*n_tot) 
            # Calculate the volatge constraints and costs + relaxation               
            cost +=  +np.array([M]*n_tot)@rho_vmax[:,t+1]+np.array([M]*n_tot)@rho_vmin[:,t+1]  # voltgage cost + relaxation
            if bool(self.scheduler_data["voltage_reference"]):
                cost += (1/2)*cp.quad_form(v[:,t+1]-v_ref, 100*W_V)
            else:
                pass
            constr += [ v[:,t+1] <= self.scheduler_data["VMAX"]+rho_vmax[:,t+1],
                        -v[:,t+1]<= -(self.scheduler_data["VMIN"]-rho_vmin[:,t+1]),
                        rho_vmax[:,t+1]<=1e6,
                        rho_vmax[:,t+1]>=0,
                        rho_vmin[:,t+1]<=1e6,
                        rho_vmin[:,t+1]>=0,
                        ]

            gen_load = R_tot@self.scheduler_data["P_DG_MAX"][t+1][self.active_tot] - R_tot@self.scheduler_data["load_forecast"][t+1][self.active_tot]
            constr += [v[:,t+1] == self.scheduler_data["V_slack"]+ gen_load + add_Q_DG + add_P_ESS + add_P_DG ]            
            

        # Constraints outside the loop    
        if self.scheduler_data["control_variables"]["ESS"]:
            constr += [SOC[:,1] == np.array(self.scheduler_data['SOC_init']), SOC[:,t+1] == np.array(self.scheduler_data['SOC_init']) ]
        constr += [v[:,1]==v_0]

        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(verbose=False)

        # print("\nThe optimal value is", problem.value)
        print("status:", problem.status)


        output = {"DG": {"reactive_power" : None, "active_power": None}, "ESS": { "active_power": None, "SOC": None}, "voltage": {"controlled":v.value, "uncontrolled":v_unctr}}
        if self.scheduler_data["control_variables"]["DG"]:
            if q_DG:
                output["DG"]["reactive_power"] = q_DG.value
            if p_DG:
                output["DG"]["active_power"] = p_DG.value
        if self.scheduler_data["control_variables"]["ESS"]:
            if p_ESS:
                output["ESS"]["active_power"] = p_ESS.value
                output["ESS"]["SOC"] = SOC.value


        return output
