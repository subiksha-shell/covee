import numpy as np

class additional():

    def __init__(self, bus_values):
        self.bus_values = bus_values


    def resize_in(self, full_nodes,active_nodes,active_power,reactive_power,active_power_ess,pvproduction,n):
        k = 0
        t = 0
        full_active = np.zeros_like(self.bus_values)
        active_power_full = np.zeros(n)
        reactive_power_full = np.zeros(n)
        active_power_ess_full = np.zeros(n)
        #activation_nodes_full = np.zeros(n)
        # pv_input_full = np.zeros(n)
        for i in self.bus_values:
            t += 1
            if any(b == full_nodes[t-1] for b in active_nodes):
                full_active[t-1] = 1
                active_power_full[t-1] = 1.0*np.array(active_power)[k]
                reactive_power_full[t-1] = reactive_power[k]
                active_power_ess_full[t-1] = 1.0*np.array(active_power_ess)[k]
                #activation_nodes_full[t-1] = activation_nodes[k]
                # pv_input_full[t-1] = pvproduction[k]
                k += 1
            else:
                full_active[t-1] = 0
                active_power_full[t-1] = 0
                reactive_power_full[t-1] = 0
                active_power_ess_full[t-1] = 0
                #activation_nodes_full[t-1] = 0
                # pv_input_full[t-1] = 0

        pv_input_full = np.zeros(n)#pv_input_full = np.zeros_like(self.bus_values)
        k = 0
        for i in range(len(active_power_full)):
            if any(b == full_nodes[i] for b in active_nodes):
                pv_input_full[i] = pvproduction[k]
                k += 1
            else:
                pv_input_full[i] = 0

        return  reactive_power_full, active_power_full,active_power_ess_full, pv_input_full, full_active


    def resize_out(self, active_nodes,q_sol_centr,p_sol_centr,p_ess_sol_centr,reactive_power_full,active_power_full, active_power_ess_full, infeasibility_output):

        active_power_sol = np.zeros_like((active_nodes),dtype = float)
        reactive_power_sol = np.zeros_like((active_nodes),dtype = float)
        active_power_ess_sol = np.zeros_like((active_nodes),dtype = float)
        k = 0
        t = 0
        if any(t != None for t in q_sol_centr):
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    reactive_power_sol[k] = q_sol_centr[t-1]
                    k += 1
                else:
                    pass
        else:
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    reactive_power_sol[k] = infeasibility_output["reactive_power"][t-1]
                    k += 1
                else:
                    pass
        k = 0
        t = 0
        if any(t != None for t in p_sol_centr):
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    active_power_sol[k] = p_sol_centr[t-1]
                    k += 1
                else:
                    pass
        else:
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    active_power_sol[k] = infeasibility_output["active_power"][t-1]
                    k += 1
                else:
                    pass
        k = 0
        t = 0
        if any(t != None for t in p_ess_sol_centr):
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    active_power_ess_sol[k] = p_ess_sol_centr[t-1]
                    k += 1
                else:
                    pass
        else:
            for i in self.bus_values:
                t +=1
                if any(b == i for b in active_nodes):
                    active_power_ess_sol[k] = infeasibility_output["active_power_ess"][t-1]
                    k += 1
                else:
                    pass

        return reactive_power_sol, active_power_sol, active_power_ess_sol

    def prioritize(self,q_sol_centr, QMIN,P_activate,case):
        n = len(q_sol_centr)        
        if case == None:
            pass
        elif case == "prioritize":
            for i in range(n):	
                if i == 0:	
                    if abs(q_sol_centr[i]) > abs(0.98*QMIN[i]):	
                        P_activate[i] = 1.0
                    else:	
                        P_activate[i] = 1e6
                    if i ==10:
                        P_activate[i] = 1.0
                elif i in range(n-1):	
                    if  ((abs(q_sol_centr[i-1]) > abs(0.98*QMIN[i-1]))) or ((abs(q_sol_centr[i+1]) > abs(0.98*QMIN[i+1]))):	
                        P_activate[i] = 1.0
                    else:	
                        P_activate[i] = 1e6	
                elif i == n-1:	
                    if ((abs(q_sol_centr[i-1]) > abs(0.98*QMIN[i-1]))): 	
                        P_activate[i] = 1.0
                    else:	
                        P_activate[i] = 1e6           	
                else:	
                    P_activate[i] = 1e6
        elif case == "test":
            for i in range(n):
                if i>10:
                    P_activate[i] = 1.0
                else:
                    P_activate[i] = 1e6

        return P_activate