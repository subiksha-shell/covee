import control_strategies.quadratic_control as quadratic_control
import numpy as np

class Quadratic_Control_PV():

    def __init__(self, grid_data, num_pv):

        self.grid_data = grid_data	
        self.num_pv = num_pv
        self.num_bus = self.grid_data["nb"]     

        self.alpha = [0.0]*len(self.num_pv)
        self.alpha_PV = [0.0]*len(self.num_pv)

    def initialize_control(self):   

        self.control_reactive_power = quadratic_control.Quadratic_Reactive_Power(self.grid_data,self.num_pv)
        self.control_reactive_power.initialize_control()

        self.control_active_power_PV = quadratic_control.Quadratic_Active_Power_PV(self.grid_data,self.num_pv)
        [self.active_power_PV, self.alpha_PV] = self.control_active_power_PV.initialize_control()

        iter_calculation = quadratic_control.iteration_calculation(self.grid_data,self.num_pv)
        self.lim = iter_calculation.calculate_alpha()

        # Set the parameters
        # ========================================================================
        self.K1 = 1.0
        for i in range(int(len(self.num_pv))):
            self.alpha[i] = self.K1*self.lim
            self.alpha_PV[i] = self.K1*self.lim
    
    def control_(self, PV_list, q_PV, active_power_PV, v_gen):
        ############# RUN QUADRATIC VOLTAGE CONTROL ###############################################
        # By changing the ration alpha/alpha_p we can control if we want use
        # more the PV or the batteries for the regulation (for example depending on the SOC)

        self.reactive_power = q_PV
        self.active_power_PV = active_power_PV
        # self.active_power_battery = active_power_battery

        # REACTIVE POWER CONTROL PV
        # ================================================================================================
        [self.reactive_power, self.mu_min] = self.control_reactive_power.Voltage_Control(PV_list, q_PV, v_gen, self.alpha)

        # # ACTIVE POWER CONTROL PV
        # # ================================================================================================
        # self.active_power_PV = self.control_active_power_PV.Voltage_Control(PV_list, active_power_PV, v_gen, self.alpha_PV)
        # for i in range(len(self.num_pv)):	
        #     if i == 0:	
        #         if self.mu_min[i+1] != 0:	
        #             self.alpha_PV[i] = self.K1*self.lim	
        #         else:	
        #             self.active_power_PV[i] = 0.0	
        #     elif i in range(len(self.num_pv)-1):	
        #         if self.mu_min[i-1] != 0 or self.mu_min[i+1] != 0:	
        #             self.alpha_PV[i] = self.K1*self.lim	
        #         else:	
        #             self.active_power_PV[i] = 0.0	
        #     elif i == len(self.num_pv)-1:	
        #         if self.mu_min[i-1] != 0: 	
        #             self.alpha_PV[i] = self.K1*self.lim	
        #         else:	
        #             self.active_power_PV[i] = 0.0               	
        #     else:	
        #         pass

        # if activate_battery == "true":
        #     # COORDINATED ACTIVE POWER CONTROL (BATT)
        #     # ==========================================================================================================================================
        #     [self.active_power_battery_list, self.active_power_battery, self.xi_min]  = self.control_active_power_batt.Voltage_Control(k, 
        #                                                                                         PV_list, self.active_power_battery, v_tot.tolist(), self.alpha_P)  
            
        #     for i in range(len(self.num_pv)):
        #         if self.mu_min[i] !=0 and i != 0:
        #             self.alpha_P[i] = self.K1*self.lim
        #         else:
        #             self.alpha_P[i] = 0.0001   
        # else:
        #     pass


        return self.reactive_power


        




