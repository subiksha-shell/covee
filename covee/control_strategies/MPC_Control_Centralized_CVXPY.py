import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
import os
import cvxpy as cp
from scipy import sparse
import coloredlogs, logging, threading

import covee.control_strategies.quadratic_control_cvxpy as quadratic_control

class MPC_Control():

    def __init__(self, grid_data, num_pv, num_ESS, control_data, scheduler_data, output_scheduler, forecast, scheduling):
        self.grid_data = grid_data	
        self.num_pv = num_pv
        self.num_ESS = num_ESS
        self.num_bus = self.grid_data["nb"]
        self.control_data = control_data
        self.scheduling = scheduling
        self.output_scheduler = output_scheduler
        self.scheduler_data = scheduler_data
        self.forecast = forecast

    def initialize_control(self): 

        self.num_pv = list(np.array(self.num_pv))
        self.bus_values = (np.array(list(range(1,self.num_bus)))).tolist()
        calculate_matrix_full = quadratic_control.matrix_calc(self.grid_data, self.bus_values) 
        [R,X] = calculate_matrix_full.calculate()
        self.additional = quadratic_control.additional(self.bus_values)
        self.reactive_power_PV = np.zeros((len(self.num_pv),self.control_data["MPC_data"]["Steps"]))#np.array([0.0]*len(self.num_pv))
        self.active_power_PV =  np.zeros((len(self.num_pv),self.control_data["MPC_data"]["Steps"]))#np.array([0.0]*len(self.num_pv))
        self.active_power_ESS =  np.zeros((len(self.num_pv),self.control_data["MPC_data"]["Steps"]))#np.array([0.0]*len(self.num_ESS))

        output_MPC = {"DG": {"reactive_power" : self.reactive_power_PV, "active_power": self.active_power_PV}, "ESS": { "active_power": self.active_power_ESS, "SOC": None}}

        # Recalculate considering only the active nodes
        full_nodes = self.bus_values
        self.active_nodes = self.num_pv
        self.active_nodes_ESS = self.num_ESS

        diff__ = list(set(self.active_nodes_ESS)- set(self.active_nodes))
        
        self.active_tot = np.sort(self.active_nodes+diff__)
        self.n = len(self.active_tot)
        calculate_matrix_full = quadratic_control.matrix_calc(self.grid_data, self.active_tot) 
        [R,X] = calculate_matrix_full.calculate()

        self.T = self.control_data["MPC_data"]["Steps"]
        

        '''
        ####################################### Create the weighted matrixes and Define the control variables ########################################
        '''
        weights = {}       
        # Create variables and matrixes DGs
        if self.control_data["control_variables"]["DG"]:
            factor_DG = 1e6
            self.factor_DG_array = np.ones((self.n))
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"Q_DG": [self.control_data["MPC_data"]["Weights"]["DG"]["reactive_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["Q_DG"][k] = factor_DG*weights["Q_DG"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_DG_array[k] = factor_DG*self.factor_DG_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints) 
                    k += 1
                self.q_DG = cp.Variable((self.n,self.T))
                self.W_Q_DG = np.diag(weights["Q_DG"])
            else:
                self.q_DG = None
            
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"P_DG": [self.control_data["MPC_data"]["Weights"]["DG"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["P_DG"][k] = factor_DG*weights["P_DG"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_DG_array[k] = factor_DG*self.factor_DG_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                self.p_DG = cp.Variable((self.n,self.T))
                self.W_P_DG = np.diag(weights["P_DG"])
            else:
                self.p_DG = None

        # Create variables and matrixes ESSs
        if self.control_data["control_variables"]["ESS"]:
            factor_ESS = 1e6
            self.factor_ESS_array = np.ones((self.n))
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                weights.update({"P_ESS": [self.control_data["MPC_data"]["Weights"]["ESS"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes_ESS):
                        pass
                    else:
                        weights["P_ESS"][k] = factor_ESS*weights["P_ESS"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_ESS_array[k] = factor_ESS*self.factor_ESS_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                self.p_ESS = cp.Variable((self.n,self.T))
                self.W_P_ESS = np.diag(weights["P_ESS"])
                if bool(self.control_data["MPC_activate"]):
                    weights.update({"SOC": [self.control_data["MPC_data"]["Weights"]["SOC"]] * self.n})
                    k = 0
                    for i in self.active_tot:
                        if any(x == i for x in self.active_nodes_ESS):
                            pass
                        else:
                            weights["SOC"][k] = factor_ESS*weights["SOC"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        k+=1                    
                    self.SOC = cp.Variable((self.n,self.T))
                    self.W_SOC = np.diag(weights["SOC"]) 
            else:
                self.p_ESS = None

        # define voltage variable and relaxation variables 
        self.v = cp.Variable((self.n,self.T))
        self.rho_vmax = cp.Variable((self.n,self.T))
        self.rho_vmin = cp.Variable((self.n,self.T))
        # add voltage in the cost
        weights.update({"V": [self.control_data["MPC_data"]["Weights"]["voltage"]]*self.n})
        self.W_V = np.diag(weights["V"])

        # define the reference values
        self.SOC_ref = {str(t): np.array([0.0]*self.n) for t in range(self.T)}
        self.p_ESS_ref = {str(t): np.array([0.0]*self.n) for t in range(self.T)}
        self.v_ref = np.array([self.scheduler_data["v_ref"]]*self.n)
        
        self.ref_iter = 0.0

        return output_MPC

    def control_(self, profiles, output, R, X, v_gen, v_ess, VMIN, VMAX, iter_MPC, output_scheduler, output_MPC, iter, reference):
        

        pvproduction = profiles["gen_profile"][iter][self.active_nodes]

        n = self.n

        v_tot = np.zeros(n)
        k_pv = 0
        k_ess = 0
        for i in range(len(self.active_tot)):
            if any(x == self.active_tot[i] for x in self.active_nodes):
                v_tot[i] = v_gen[k_pv]
                k_pv +=1
            if any(x == self.active_tot[i] for x in self.active_nodes_ESS):
                v_tot[i] = v_ess[k_ess]
                k_ess +=1
            else:
                pass

        # Resize the power vectors to consider the full voltage vector in the equation
        [output_full, pv_input_full] = self.additional.resize_in(self.active_tot, self.active_nodes, self.active_nodes_ESS, output,pvproduction,n, self.control_data)
        reference = self.additional.resize_reference(self.control_data, self.active_tot, self.active_nodes, reference, self.T)

        ###  LIMITS AND PARAMETERS ###
        k = 0
        var= {}
        for t in range(self.T):
            if t == 0:
                var["QMIN"] = {str(t+1): np.array([-0.312*pv_input_full[i] for i in range(int(n))])}
                var["QMAX"] = {str(t+1): np.array([0.312*pv_input_full[i] for i in range(int(n))])}
                var["PMIN"] = {str(t+1): np.array([-(pv_input_full[i]+1e-6) for i in range(int(n))])}
                var["PMAX"] = {str(t+1): np.array([0.0+1e-6 for i in range(int(n))])}
                var["P_ESS_MIN"] = {str(t+1): np.array([-0.312 for i in range(int(n))])}
                var["P_ESS_MAX"] = {str(t+1): np.array([0.312 for i in range(int(n))])}
                var["SOC_MIN"] = {str(t+1): np.array([20.0 for i in range(int(n))])}
                var["SOC_MAX"] = {str(t+1): np.array([90.0 for i in range(int(n))])}
                var["SOC"] = {str(t+1): np.array(output["ESS"]["SOC"])} # Real measurement for time t=0
                var["P_MEAS"] = np.array(output["ESS"]["active_power"])
            else:
                var["QMIN"].update({str(t+1): -0.312*np.array([self.forecast['gen_forecast'][t+iter_MPC,i] for i in range(int(n))])}) 
                var["QMAX"].update({str(t+1): 0.312*np.array([self.forecast['gen_forecast'][t+iter_MPC,i] for i in range(int(n))])})
                var["PMIN"].update({str(t+1): np.array([-(self.forecast['gen_forecast'][t+iter_MPC,i]+1e-6) for i in range(int(n))])})
                var["PMAX"].update({str(t+1): np.array([0.0+1e-6 for i in range(int(n))])})
                var["P_ESS_MIN"].update({str(t+1): np.array([-0.312 for i in range(int(n))])})
                var["P_ESS_MAX"].update({str(t+1): np.array([0.312 for i in range(int(n))])})
                var["SOC_MIN"].update({str(t+1): np.array([20.0 for i in range(int(n))])})
                var["SOC_MAX"].update({str(t+1): np.array([90.0 for i in range(int(n))])})


        diff_DG = list(set(self.active_tot)- set(self.active_nodes))
        diff_ESS = list(set(self.active_tot)- set(self.active_nodes_ESS))

        var["VMAX"] = [VMAX] * int(n)       # create array of VMAX
        var["VMIN"] = [VMIN] * int(n)       # create array of VMIN  

        if bool(self.scheduling["Activate"]):
            for t in range(self.T):
                self.SOC_ref[str(t)] = np.array(output_scheduler["ESS"]["SOC"][:,int(self.ref_iter)+t])
                self.p_ESS_ref[str(t)] = np.array(output_scheduler["ESS"]["active_power"][:,int(self.ref_iter)+t])
            self.ref_iter+=1
        
        if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
            self.SOC[:,0].value == np.array(output["ESS"]["SOC"])

        '''
        ########################################################################################################################
        '''
        # Trasform Matrixes in arrays
        R_tot = np.array(R)
        X_tot = np.array(X)

        # Calculate uncontrolled voltage and the predictions
        v_N = {str(t): np.array([0.0]*self.n) for t in range(self.T)}
        for t in range(self.T):
            if t == 0:
                v_N[str(t)] = np.array(v_tot) 
                if self.control_data["control_variables"]["DG"]:
                    if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                        v_N[str(t)] += - X_tot@np.array(output_full["DG"]["reactive_power"])
                    if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                        v_N[str(t)] += - R_tot@np.array(output_full["DG"]["active_power"])
                
                if self.control_data["control_variables"]["ESS"]:
                    if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                        v_N[str(t)] += - R_tot@np.array(output_full["ESS"]["active_power"])  
            else:
                v_N[str(t)] = self.scheduler_data["V_slack"] + R_tot@self.scheduler_data["P_DG_MAX"][t][self.active_tot] \
                                        - R_tot@self.scheduler_data["load_forecast"][t][self.active_tot]



        '''
        ############################ Start the optimization #########################################
        '''
        cost = 0
        constr = []  

        for t in range(self.T-1):
            if self.control_data["control_variables"]["DG"]:
                # Calculate the DG reactive power constraints and costs   
                if self.q_DG:
                    constr += [ cp.multiply(self.factor_DG_array,self.q_DG[:,t+1]) <= var["QMAX"][str(t+1)], 
                                -cp.multiply(self.factor_DG_array,self.q_DG[:,t+1]) <= -var["QMIN"][str(t+1)],
                            ]
                    cost += (1/2)*cp.quad_form(self.q_DG[:,t+1], self.W_Q_DG)
                    # contribution to voltage regulation
                    add_Q_DG = X_tot@self.q_DG[:,t+1]                
                else:
                    add_Q_DG = np.array([0.0]*n) 

                # Calculate the DG active power constraints and costs 
                if self.p_DG:
                    constr += [ cp.multiply(self.factor_DG_array,self.p_DG[:,t+1]) <= var["PMAX"][str(t+1)], 
                                -cp.multiply(self.factor_DG_array,self.p_DG[:,t+1]) <= -var["PMIN"][str(t+1)],
                            ]
                    cost += (1/2)*cp.quad_form(self.p_DG[:,t+1]-reference["DG"]["active_power"][str(t+1)], self.W_P_DG)
                    # contribution to voltage regulation
                    add_P_DG = R_tot@self.p_DG[:,t+1]               
                else:
                    add_P_DG = np.array([0.0]*n) 
            else:
                add_Q_DG = np.array([0.0]*n) 
                add_P_DG = np.array([0.0]*n) 

            if self.control_data["control_variables"]["ESS"]:
                # Calculate the DG active power constraints and costs 
                if self.p_ESS:
                    constr += [ cp.multiply(self.factor_ESS_array,self.p_ESS[:,t+1]) <= var["P_ESS_MAX"][str(t+1)],
                                -cp.multiply(self.factor_ESS_array,self.p_ESS[:,t+1]) <= -var["P_ESS_MIN"][str(t+1)]
                            ]                   
                    constr += [
                                self.SOC[:,t+1]<= var["SOC_MAX"][str(t+1)],
                                -self.SOC[:,t+1] <= -var["SOC_MIN"][str(t+1)],
                                self.SOC[:,t+1] == self.SOC[:,t] - self.control_data["factor_SOC"]*self.p_ESS[:,t+1]
                            ]
                       
                    if bool(self.scheduling["Activate"]):
                        cost += (1/2)*cp.quad_form(self.SOC[:,t+1]-self.SOC_ref[str(t+1)], 10*self.W_SOC)
                    else:
                        pass

                    cost += (1/2)*cp.quad_form(self.p_ESS[:,t+1], self.W_P_ESS)
                    # contribution to voltage regulation
                    add_P_ESS = R_tot@self.p_ESS[:,t+1]               
                else:
                    add_P_ESS = np.array([0.0]*n) 
            else:
                add_P_ESS = np.array([0.0]*n) 

            # # Calculate the volatge constraints and costs + relaxation     
            M = self.control_data["M"]          
            cost +=  +np.array([M]*n)@self.rho_vmax[:,t+1]+np.array([M]*n)@self.rho_vmin[:,t+1] # voltgage cost + relaxation
            if bool(self.control_data["voltage_reference"]):
                cost += (1/2)*cp.quad_form(self.v[:,t+1]-self.v_ref, 1000*self.W_V)
            else:
                pass
            constr += [                 	
                        self.v[:,t+1] <= var["VMAX"]+self.rho_vmax[:,t+1],
                        self.v[:,t+1]>= var["VMIN"]-self.rho_vmin[:,t+1],
                        self.rho_vmax[:,t+1]>=0,
                        self.rho_vmin[:,t+1]>=0,
                        ]
            
            constr += [self.v[:,t+1] == v_N[str(t)] + add_Q_DG + add_P_ESS + add_P_DG ] 

        ## Constraints outside the loop    
        if bool(self.scheduling["Activate"]) and any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
            constr += [self.SOC[:,0] == self.SOC_ref[str(0)]]#
        constr += [self.v[:,0]==v_N[str(0)]]

        # solve the problem
        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(verbose=False, solver = cp.OSQP)

        # print("\nThe optimal value is", problem.value)
        print("status:", problem.status)

        if self.control_data["control_variables"]["DG"]:
            if self.q_DG:
                output_MPC["DG"]["reactive_power"] = self.q_DG.value
            if self.p_DG:
                output_MPC["DG"]["active_power"] = self.p_DG.value
        if self.control_data["control_variables"]["ESS"]:
            if self.p_ESS:
                output_MPC["ESS"]["active_power"] = self.p_ESS.value
                output_MPC["ESS"]["SOC"] = self.SOC.value

        return output_MPC