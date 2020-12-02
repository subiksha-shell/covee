import control_strategies.quadratic_control as quadratic_control
import numpy as np

class Quadratic_Control_ESS():

    def __init__(self, grid_data, num_ESS):

        self.grid_data = grid_data	
        self.num_ESS = num_ESS
        self.num_bus = self.grid_data["nb"]     

        self.alpha_ESS = [0.0]*len(self.num_ESS)

    def initialize_control(self): 

        self.control_active_power_ESS = quadratic_control.Quadratic_Active_Power_Batt(self.grid_data,self.num_ESS)
        [self.active_power_ESS, self.alpha_ESS] = self.control_active_power_ESS.initialize_control()

        iter_calculation = quadratic_control.iteration_calculation(self.grid_data,self.num_ESS)
        self.lim = iter_calculation.calculate_alpha()

        # Set the parameters
        # ========================================================================
        self.K1 = 1.0
        for i in range(int(len(self.num_ESS))):
            self.alpha_ESS[i] = self.K1*self.lim
    
    def control_(self, active_power_battery, v_ess):
        ############# RUN QUADRATIC VOLTAGE CONTROL ###############################################
        # By changing the ration alpha/alpha_p we can control if we want use
        # more the PV or the batteries for the regulation (for example depending on the SOC)

        self.active_power_battery = active_power_battery

        # COORDINATED ACTIVE POWER CONTROL (BATT)
        # ==========================================================================================================================================
        [self.active_power_battery, self.xi_min]  = self.control_active_power_ESS.Voltage_Control(self.active_power_battery, v_ess, self.alpha_ESS)  
        
        return self.active_power_battery


        




