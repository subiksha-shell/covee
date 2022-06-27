import covee.control_strategies.quadratic_control_dualascent as quadratic_control
import numpy as np

class Quadratic_Control():

    def __init__(self, grid_data, num_pv, num_ESS, control_data):

        self.grid_data = grid_data	
        self.num_pv = num_pv
        self.num_bus = self.grid_data["nb"]

        self.grid_data = grid_data	
        self.num_ESS = num_ESS
        self.num_bus = self.grid_data["nb"]     

        self.alpha_ESS = [0.0]*len(self.num_ESS)     
        self.alpha = [0.0]*len(self.num_pv)
        self.alpha_PV = [0.0]*len(self.num_pv)

        self.control_data = control_data

    def initialize_control(self): 

        self.control_reactive_power = quadratic_control.Quadratic_Reactive_Power(self.grid_data,self.num_pv)
        [self.reactive_power, self.mu_min,Z] = self.control_reactive_power.initialize_control()

        self.control_active_power_PV = quadratic_control.Quadratic_Active_Power_PV(self.grid_data,self.num_pv)
        [self.active_power_PV, self.alpha_PV,Z] = self.control_active_power_PV.initialize_control()

        self.control_active_power_ESS = quadratic_control.Quadratic_Active_Power_Batt(self.grid_data,self.num_ESS)
        [self.active_power_ESS, self.alpha_ESS, self.xi_min,R_ess] = self.control_active_power_ESS.initialize_control()

        iter_calculation = quadratic_control.iteration_calculation(self.grid_data,self.num_pv)
        self.lim = iter_calculation.calculate_alpha()

        iter_calculation = quadratic_control.iteration_calculation(self.grid_data,self.num_ESS)
        self.lim2 = iter_calculation.calculate_alpha()

        # Set the parameters
        # ========================================================================
        self.K1 = 1.0
        for i in range(int(len(self.num_pv))):
            self.alpha[i] = self.K1*self.lim
            self.alpha_PV[i] = self.K1*self.lim
        self.K2 = 1.0
        for i in range(int(len(self.num_ESS))):
            self.alpha_ESS[i] = self.K2*self.lim2

        R = np.real(Z)
        X = np.imag(Z)

        output = {"DG": {"reactive_power" : self.reactive_power, "active_power": self.active_power_PV}, "ESS": { "active_power": self.active_power_ESS, "SOC": None}}
        
        return X,R, output
    
    def control_(self, PV_list, output, R, X, v_gen, v_ess, VMIN,VMAX, iter):
        ############# RUN QUADRATIC VOLTAGE CONTROL ###############################################
        # By changing the ration alpha/alpha_p we can control if we want use
        # more the PV or the batteries for the regulation (for example depending on the SOC)

        # CONTROL PV
        # ================================================================================================
        if self.control_data["control_variables"]["DG"]:
            if any(i == "reactive_power" for i in self.control_data["control_variables"]["DG"]):
                # ==========================================================================================================================================
                [reactive_power_output, self.mu_min] = self.control_reactive_power.Voltage_Control(PV_list, output["DG"]["reactive_power"], v_gen, self.alpha, VMIN ,VMAX)
                output["DG"]["reactive_power"] = reactive_power_output
            else:
                pass
        
            if any(i == "active_power" for i in self.control_data["control_variables"]["DG"]):
                # ==========================================================================================================================================
                active_power_PV_output = self.control_active_power_PV.Voltage_Control(PV_list, output["DG"]["active_power"], v_gen, self.alpha_PV, VMIN ,VMAX)
                output["DG"]["active_power"] = active_power_PV_output
            else:
                pass

            for i in range(len(self.num_pv)):	
                if i == 0:	
                    if self.mu_min[i+1] != 0:	
                        self.alpha_PV[i] = 0.8*self.K1*self.lim		
                    else:	
                        self.alpha_PV[i] = 1e-3
                elif i in range(len(self.num_pv)-1):	
                    if (self.mu_min[i-1] != 0) or (self.mu_min[i+1] != 0):	
                        self.alpha_PV[i] = 0.8*self.K1*self.lim	
                    else:	
                        self.alpha_PV[i] = 1e-3	
                elif i == len(self.num_pv)-1:	
                    if self.mu_min[i-1] != 0: 	
                        self.alpha_PV[i] = 0.8*self.K1*self.lim	
                    else:	
                        self.alpha_PV[i] = 1e-3           	
                else:	
                    pass
        else:
            pass

        # CONTROL ESS
        # ================================================================================================
        if self.control_data["control_variables"]["ESS"]:           
            if any(i == "active_power" for i in self.control_data["control_variables"]["ESS"]):          
                # ==========================================================================================================================================
                [active_power_battery_output, self.xi_min]  = self.control_active_power_ESS.Voltage_Control(output["ESS"]["active_power"], v_ess, self.alpha_ESS, VMIN ,VMAX)
                output["ESS"]["active_power"] = active_power_battery_output
            else:
                pass
        else:
            pass

        return output


        




