import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
import os
import cvxpy as cp
from scipy import sparse
import coloredlogs, logging, threading

import control_strategies.quadratic_control as quadratic_control

class Quadratic_Control():

    def __init__(self, grid_data, num_pv, num_ESS, control_data):
        self.grid_data = grid_data	
        self.num_pv = num_pv
        self.num_ESS = num_ESS
        self.num_bus = self.grid_data["nb"]
        self.control_data = control_data     


        self.QMIN = []
        self.QMAX = []
        self.PMIN = []
        self.PMAX = []

        # Problem parameters
        # =============================================================
        self.V_MIN = 0.95  # undervoltage limit
        self.V_MAX = 1.05  # overvoltage limit
        self.V_NOM = 1.00  # nominal voltage

        # DEFINE LIM
        # =============================================================
        for i in range(int(len(self.num_pv))):
            self.QMIN.append(-0.8)
            self.QMAX.append(0.8)
            self.PMIN.append(-3.0)
            self.PMAX.append(+3.0)

    def initialize_control(self): 

        self.num_pv = list(np.array(self.num_pv))
        self.bus_values = (np.array(list(range(1,self.num_bus)))).tolist()
        calculate_matrix_full = quadratic_control.matrix_calc(self.grid_data, self.bus_values) 
        [R,X] = calculate_matrix_full.calculate()
        self.additional = quadratic_control.additional(self.bus_values)

        self.P_activate = [1e6]*len(self.bus_values)
      
        return R,X

    def control_(self, pvproduction, active_power, reactive_power, R, X, active_nodes, active_nodes_ESS, v_tot, active_power_ess, v_ess, SOC):

        full_nodes = self.bus_values
        n = len(self.bus_values)   

        # Resize the power vectors to consider the full voltage vector in the equation
        [reactive_power_full, active_power_full, pv_input_full, full_active] = self.additional.resize_in(full_nodes,active_nodes,active_power,
                                                                                                            reactive_power,pvproduction,n)
        [reactive_power_ess_full, active_power_ess_full, no_output, full_active] = self.additional.resize_in(full_nodes,active_nodes_ESS,active_power_ess,
                                                                                                            None,pvproduction,n)

        k = 0
        var= {}
        var["ref"] =  {"active_power": np.array([0.0]*n), "voltage": np.array([-0.0]*n)}
        var["QMIN"] = np.array([-0.312*pv_input_full[i] for i in range(int(n))])
        var["QMAX"] = np.array([0.312*pv_input_full[i] for i in range(int(n))])
        var["PMIN"] = np.array([-(pv_input_full[i]+1e-6) for i in range(int(n))])
        var["PMAX"] = np.array([0.0+1e-6 for i in range(int(n))])
        var["P_ESS_MIN"] = np.array([-0.312 for i in range(int(n))])
        var["P_ESS_MAX"] = np.array([0.312 for i in range(int(n))])
        var["SOC_MIN"] = np.array([20.0 for i in range(int(n))])
        var["SOC_MAX"] = np.array([90.0 for i in range(int(n))])
        var["VNOM"] = {"active_power": np.transpose(np.matrix(v_tot))-R*np.transpose(np.matrix([active_power_full])), 
                        "reactive_power": np.transpose(np.matrix(v_tot))-X*np.transpose(np.matrix([reactive_power_full]))}
        var["SOC"] = np.array(SOC)


        diff_DG = list(set(full_nodes)- set(active_nodes))
        diff_ESS = list(set(full_nodes)- set(active_nodes_ESS))

        var["VMAX"] = [self.V_MAX] * int(n)       # create array of VMAX
        var["VMIN"] = [self.V_MIN] * int(n)       # create array of VMIN    

        '''
        ####################################### Create the weighted matrixes #########################################################################
        '''
        weights = {}       
        # Create variables and matrixes DGs
        if self.control_data["control_variables"]["DG"]:
            factor_DG = 1e6
            factor_DG_array = np.ones((n))
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"Q_DG": [1] * n})
                for i in diff_DG:
                    weights["Q_DG"][i] = factor_DG*weights["Q_DG"][i]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                    factor_DG_array[i] = factor_DG*factor_DG_array[i]  # To esclude nodes where no DGs are instualled (later in the constraints)
                q_DG = cp.Variable((n))
                W_Q_DG = np.diag(weights["Q_DG"])
            else:
                q_DG = None
            
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                weights.update({"P_DG": [1] * n})
                for i in diff_DG:
                    weights["P_DG"][i] = factor_DG*weights["P_DG"][i]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                    factor_DG_array[i] = factor_DG*factor_DG_array[i]  # To esclude nodes where no DGs are instualled (later in the constraints)
                p_DG = cp.Variable((n))
                W_P_DG = np.diag(weights["P_DG"])
            else:
                p_DG = None

        # Create variables and matrixes ESSs
        if self.control_data["control_variables"]["ESS"]:
            factor_ESS = 1e6
            factor_ESS_array = np.ones((n))
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                weights.update({"P_ESS": [1] * n})
                for i in diff_DG:
                    weights["P_ESS"][i] = factor_ESS*weights["P_ESS"][i]  # To esclude nodes where no DGs are instualled (in the matrix weights)
                    factor_ESS_array[i] = factor_ESS*factor_ESS_array[i]  # To esclude nodes where no DGs are instualled (later in the constraints)
                p_ESS = cp.Variable((n))
                W_P_ESS = np.diag(weights["P_ESS"])
                SOC = cp.Variable((n)) 
            else:
                p_ESS = None

        '''
        ########################################### Define the control variables #############################################################################
        '''
        # define voltage variable and relaxation variables 
        v = cp.Variable((n))
        rho_vmax = cp.Variable((n))
        rho_vmin = cp.Variable((n))
        # add voltage in the cost
        weights.update({"V": [1] * n})
        W_V = np.diag(weights["V"])

        # Trasform Matrixes in arrays
        R_tot = np.array(R)
        X_tot = np.array(X)

        # Calculate uncontrolled voltage
        v_N = np.array(v_tot) 
        if self.control_data["control_variables"]["DG"]:
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                v_N += - X_tot@np.array(reactive_power_full)
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                v_N += - R_tot@np.array(active_power_full)
        if self.control_data["control_variables"]["ESS"]:
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):
                v_N += - R_tot@np.array(active_power_ess_full)  

        '''
        ############################ Start the optimization #########################################
        '''
        cost = 0
        constr = []      

        # Calculate the DG reactive power constraints and costs   
        if q_DG:
            constr += [ cp.multiply(factor_DG_array,q_DG[:]) <= var["QMAX"], 
                        -cp.multiply(factor_DG_array,q_DG[:]) <= -var["QMIN"],
                    ]
            cost += (1/2)*cp.quad_form(q_DG[:], W_Q_DG)
            # contribution to voltage regulation
            add_Q_DG = X_tot@q_DG[:]                
        else:
            add_Q_DG = np.array([0.0]*n) 

        # Calculate the DG active power constraints and costs 
        if p_DG:
            constr += [ cp.multiply(factor_DG_array,p_DG[:]) <= var["PMAX"], 
                        -cp.multiply(factor_DG_array,p_DG[:]) <= -var["PMIN"],
                    ]
            cost += (1/2)*cp.quad_form(p_DG[:], W_P_DG)
            # contribution to voltage regulation
            add_P_DG = R_tot@p_DG[:]               
        else:
            add_P_DG = np.array([0.0]*n) 

        # Calculate the DG active power constraints and costs 
        if p_ESS:
            constr += [ cp.multiply(factor_ESS_array,p_ESS[:]) <= var["P_ESS_MAX"],
                        -cp.multiply(factor_ESS_array,p_ESS[:]) <= -var["P_ESS_MIN"],
                        SOC[:] <= var["SOC_MAX"]
                        -SOC[:] <= -var["SOC_MIN"]
                    ]
            constr += [SOC[:] == var["SOC"] - 0.2*p_ESS[:] ]

            cost += (1/2)*cp.quad_form(p_ESS[:], W_P_ESS)
            # contribution to voltage regulation
            add_P_ESS = R_tot@p_ESS[:]               
        else:
            add_P_ESS = np.array([0.0]*n) 


        M = self.control_data["M"]
        v_ref = np.array([self.control_data["v_ref"]]*n)
        # Calculate the volatge constraints and costs + relaxation               
        cost +=  np.array([M]*n)@rho_vmax[:]+np.array([M]*n)@rho_vmin[:]+(1/2)*cp.quad_form(v[:]-v_ref, 100*W_V)  # voltgage cost + relaxation
        constr += [ v[:] <= var["VMAX"]+rho_vmax[:],
                    -v[:]<= -(var["VMIN"]-rho_vmin[:]),
                    rho_vmax[:]<=1e6,
                    rho_vmax[:]>=0,
                    rho_vmin[:]<=1e6,
                    rho_vmin[:]>=0,
                    ]
        constr += [v[:] == v_N + add_Q_DG + add_P_ESS + add_P_DG ]            

        # solve the problem
        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(verbose=False)

        # print("\nThe optimal value is", problem.value)
        print("status:", problem.status)


        output = {"DG": {"reactive_power" : None, "active_power": None}, "ESS": { "active_power": None, "SOC": None}, "voltage": {"controlled":v.value, "uncontrolled":v_N}}
        if q_DG:
            output["DG"]["reactive_power"] = q_DG.value
        if p_DG:
            output["DG"]["active_power"] = p_DG.value
        if p_ESS:
            output["ESS"]["active_power"] = p_ESS.value
            output["ESS"]["SOC"] = SOC.value

        return output