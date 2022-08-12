import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
import os
import cvxpy as cp
from scipy import sparse
import coloredlogs, logging, threading

import covee.control_strategies.quadratic_control_cvxpy as quadratic_control

class Quadratic_Control():

    def __init__(self, grid_data, num_pv, num_ESS, conf_dict, control_data, scheduling):
        self.grid_data = grid_data	
        self.num_pv = num_pv
        self.num_ESS = num_ESS
        self.num_bus = self.grid_data["nb"]
        self.conf_dict = conf_dict
        self.control_data = control_data
        self.scheduling = scheduling

    def initialize_control(self): 

        self.num_pv = list(np.array(self.num_pv))
        self.bus_values = (np.array(list(range(1,self.num_bus)))).tolist()
        calculate_matrix_full = quadratic_control.matrix_calc(self.grid_data, self.bus_values) 
        [R,X] = calculate_matrix_full.calculate()
        self.additional = quadratic_control.additional(self.bus_values)
        self.reactive_power_PV = np.array([0.0]*len(self.num_pv))
        self.active_power_PV = np.array([0.0]*len(self.num_pv))
        self.active_power_ESS = np.array([0.0]*len(self.num_ESS))

        output = {"DG": {"reactive_power" : self.reactive_power_PV, "active_power": self.active_power_PV}, "ESS": { "active_power": self.active_power_ESS, "SOC": None}}

        # Recalculate considering only the active nodes
        full_nodes = self.bus_values
        self.active_nodes = self.num_pv
        self.active_nodes_ESS = self.num_ESS

        diff__ = list(set(self.active_nodes_ESS)- set(self.active_nodes))
        
        self.active_tot = np.sort(self.active_nodes+diff__)
        self.n = len(self.active_tot)
        calculate_matrix_full = quadratic_control.matrix_calc(self.grid_data, self.active_tot) 
        [R,X] = calculate_matrix_full.calculate()
        

        '''
        ####################################### Create the weighted matrixes and Define the control variables ########################################
        '''
        weights = {}       
        # Create variables and matrixes DGs
        if self.control_data["control_variables"]["DG"]:
            factor_DG = 1e6
            self.factor_DG_array = np.ones((self.n))
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"Q_DG": [self.control_data["Weights_CVXPY"]["DG"]["reactive_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["Q_DG"][k] = factor_DG*weights["Q_DG"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_DG_array[k] = factor_DG*self.factor_DG_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints) 
                    k += 1
                self.q_DG = cp.Variable((self.n))
                self.W_Q_DG = np.diag(weights["Q_DG"])
                self.q_DG_ref = np.array([0.0]*self.n) 
            else:
                self.q_DG = None
            
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"P_DG": [self.control_data["Weights_CVXPY"]["DG"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes):
                        pass
                    else:
                        weights["P_DG"][k] = factor_DG*weights["P_DG"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_DG_array[k] = factor_DG*self.factor_DG_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                self.p_DG = cp.Variable((self.n))
                self.W_P_DG = np.diag(weights["P_DG"])
                self.p_DG_ref = np.array([0.0]*self.n) 
            else:
                self.p_DG = None

        # Create variables and matrixes ESSs
        if self.control_data["control_variables"]["ESS"]:
            factor_ESS = 1e6
            self.factor_ESS_array = np.ones((self.n))
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                weights.update({"P_ESS": [self.control_data["Weights_CVXPY"]["ESS"]["active_power"]] * self.n})
                k = 0
                for i in self.active_tot:
                    if any(x == i for x in self.active_nodes_ESS):
                        pass
                    else:
                        weights["P_ESS"][k] = factor_ESS*weights["P_ESS"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        self.factor_ESS_array[k] = factor_ESS*self.factor_ESS_array[k]  # To esclude nodes where no DGs are instualled (later in the constraints)
                    k+=1
                self.p_ESS = cp.Variable((self.n))
                self.W_P_ESS = np.diag(weights["P_ESS"])
                self.p_ESS_ref = np.array([0.0]*self.n) 
                if bool(self.scheduling["Activate"]):
                    weights.update({"SOC": [self.control_data["Weights_CVXPY"]["SOC"]] * self.n})
                    k = 0
                    for i in self.active_tot:
                        if any(x == i for x in self.active_nodes_ESS):
                            pass
                        else:
                            weights["SOC"][k] = factor_ESS*weights["SOC"][k]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                        k+=1
                    self.W_SOC = np.diag(weights["SOC"])
                    self.SOC_ref = np.array([0.0]*self.n)
                self.SOC = cp.Variable((self.n))
                self.p_ESS_ref = np.array([0.0]*self.n)                 
            else:
                self.p_ESS = None

        # define voltage variable and relaxation variables 
        self.v = cp.Variable((self.n))
        self.rho_vmax = cp.Variable((self.n))
        self.rho_vmin = cp.Variable((self.n))
        # add voltage in the cost
        weights.update({"V": [1] * self.n})
        self.v_ref = np.array([self.scheduling["scheduler_data"]["v_ref"]]*self.n)
        self.W_V = np.diag(weights["V"])

        

        return R,X,output

    def control_(self, profiles, output, R, X, v_gen, v_ess, VMIN, VMAX, iter, iter_MPC, output_MPC):
        

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

        ###  LIMITS AND PARAMETERS ###
        k = 0
        var= {}
        var["QMIN"] = np.array([-0.312*pv_input_full[i] for i in range(int(n))])
        var["QMAX"] = np.array([0.312*pv_input_full[i] for i in range(int(n))])
        var["PMIN"] = np.array([-(pv_input_full[i]+1e-6) for i in range(int(n))])
        var["PMAX"] = np.array([0.0+1e-6 for i in range(int(n))])
        var["P_ESS_MIN"] = np.array([-0.312 for i in range(int(n))])
        var["P_ESS_MAX"] = np.array([0.312 for i in range(int(n))])
        var["SOC_MIN"] = np.array([20.0 for i in range(int(n))])
        var["SOC_MAX"] = np.array([90.0 for i in range(int(n))])
        var["SOC"] = np.array(output["ESS"]["SOC"])


        diff_DG = list(set(self.active_tot)- set(self.active_nodes))
        diff_ESS = list(set(self.active_tot)- set(self.active_nodes_ESS))

        var["VMAX"] = [VMAX] * int(n)       # create array of VMAX
        var["VMIN"] = [VMIN] * int(n)       # create array of VMIN  

        if bool(self.conf_dict["CONTROL_DATA"]["MPC_activate"]):
            
            if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
                self.p_DG_ref = np.array(output_MPC["DG"]["active_power"][:,1])
            if any(x =="reactive_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
                self.q_DG_ref = np.array(output_MPC["DG"]["reactive_power"][:,1])
            if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]):
                self.p_ESS_ref = np.array(output_MPC["ESS"]["active_power"][:,1])
                self.SOC_ref = np.array(output_MPC["ESS"]["SOC"][:,1])

        '''
        ########################################################################################################################
        '''
        # Trasform Matrixes in arrays
        R_tot = np.array(R)
        X_tot = np.array(X)

        # Calculate uncontrolled voltage
        v_N = np.array(v_tot) 
        if self.control_data["control_variables"]["DG"]:
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                v_N += - X_tot@np.array(output_full["DG"]["reactive_power"])
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                v_N += - R_tot@np.array(output_full["DG"]["active_power"])
        
        if self.control_data["control_variables"]["ESS"]:
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                v_N += - R_tot@np.array(output_full["ESS"]["active_power"])  

        '''
        ############################ Start the optimization #########################################
        '''
        cost = 0
        constr = []      

        if self.control_data["control_variables"]["DG"]:
            # Calculate the DG reactive power constraints and costs   
            if self.q_DG:
                constr += [ cp.multiply(self.factor_DG_array,self.q_DG[:]) <= var["QMAX"], 
                            -cp.multiply(self.factor_DG_array,self.q_DG[:]) <= -var["QMIN"],
                        ]
                if bool(self.conf_dict["CONTROL_DATA"]["MPC_activate"]):
                    cost += (1/2)*cp.quad_form(self.q_DG[:]-self.q_DG_ref, self.W_Q_DG)
                else:
                    cost += (1/2)*cp.quad_form(self.q_DG[:], self.W_Q_DG)
                # contribution to voltage regulation
                add_Q_DG = X_tot@self.q_DG[:]                
            else:
                add_Q_DG = np.array([0.0]*n) 

            # Calculate the DG active power constraints and costs 
            if self.p_DG:
                constr += [ cp.multiply(self.factor_DG_array,self.p_DG[:]) <= var["PMAX"], 
                            -cp.multiply(self.factor_DG_array,self.p_DG[:]) <= -var["PMIN"],
                        ]
                if bool(self.conf_dict["CONTROL_DATA"]["MPC_activate"]):
                    cost += (1/2)*cp.quad_form(self.p_DG[:]-self.p_DG_ref, self.W_P_DG)
                else:
                    cost += (1/2)*cp.quad_form(self.p_DG[:], self.W_P_DG)
                # contribution to voltage regulation
                add_P_DG = R_tot@self.p_DG[:]               
            else:
                add_P_DG = np.array([0.0]*n) 
        else:
            add_Q_DG = np.array([0.0]*n) 
            add_P_DG = np.array([0.0]*n) 

        if self.control_data["control_variables"]["ESS"]:
            # Calculate the DG active power constraints and costs 
            if self.p_ESS:
                constr += [ cp.multiply(self.factor_ESS_array,self.p_ESS[:]) <= var["P_ESS_MAX"],
                            -cp.multiply(self.factor_ESS_array,self.p_ESS[:]) <= -var["P_ESS_MIN"]
                        ]
                constr += [
                            self.SOC[:] <= var["SOC_MAX"],
                            -self.SOC[:] <= -var["SOC_MIN"],
                            self.SOC[:] == self.SOC_ref[:] - self.control_data["factor_SOC"]*(self.p_ESS[:]-self.p_ESS_ref)
                        ]
                if bool(self.conf_dict["CONTROL_DATA"]["MPC_activate"]):
                    cost += (1/2)*cp.quad_form(self.p_ESS[:]-self.p_ESS_ref, self.W_P_ESS)
                else:
                    cost += (1/2)*cp.quad_form(self.p_ESS[:], self.W_P_ESS)
                # contribution to voltage regulation
                add_P_ESS = R_tot@self.p_ESS[:]               
            else:
                add_P_ESS = np.array([0.0]*n) 
        else:
            add_P_ESS = np.array([0.0]*n) 

        # # Calculate the volatge constraints and costs + relaxation     
        M = self.control_data["M"]          
        cost +=  +np.array([M]*n)@self.rho_vmax[:]+np.array([M]*n)@self.rho_vmin[:] # voltgage cost + relaxation
        constr += [                 	
                    self.v[:] <= var["VMAX"]+self.rho_vmax[:],
                    self.v[:]>= var["VMIN"]-self.rho_vmin[:],
                    self.rho_vmax[:]>=0,
                    self.rho_vmin[:]>=0,
                    ]
        
        constr += [self.v[:] == v_N + add_Q_DG + add_P_ESS + add_P_DG ] 

        # if bool(self.control_data["voltage_reference"]):
        #     cost+=(1/2)*cp.quad_form(self.v[:]-self.v_ref, 1000*self.W_V)
        # else:
        #     pass

        if self.control_data["online_activate"]:
            # solve the problem
            problem = cp.Problem(cp.Minimize(cost), constr)
            problem.solve(verbose=False, solver = cp.OSQP)

            # print("\nThe optimal value is", problem.value)
            print("status:", problem.status)
            if self.control_data["control_variables"]["DG"]:
                if self.q_DG:
                    output["DG"]["reactive_power"] = self.q_DG.value
                if self.p_DG:
                    output["DG"]["active_power"] = self.p_DG.value
            if self.control_data["control_variables"]["ESS"]:
                if self.p_ESS:
                    output["ESS"]["active_power"] = self.p_ESS.value
                    output["ESS"]["SOC"] = self.SOC.value#
        else:
            if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
                output["DG"]["active_power"] = np.array(output_MPC["DG"]["active_power"][:,1])
            if any(x =="reactive_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["DG"]):
                output["DG"]["reactive_power"] = np.array(output_MPC["DG"]["reactive_power"][:,1])
            if any(x =="active_power" for x in self.conf_dict["CONTROL_DATA"]["control_variables"]["ESS"]):
                output["ESS"]["active_power"] = np.array(output_MPC["ESS"]["active_power"][:,1])
                output["ESS"]["SOC"] = np.array(output_MPC["ESS"]["SOC"][:,1])               

        return output