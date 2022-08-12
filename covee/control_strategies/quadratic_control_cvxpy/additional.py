import numpy as np

class additional():

    def __init__(self, bus_values):
        self.bus_values = bus_values


    def resize_in(self, full_nodes,active_nodes,active_ESS,output,pvproduction,n,control_data):
        
        for k in control_data["control_variables"]["DG"]:
            s = 0
            t = 0
            if k:
                full = np.zeros(len(full_nodes))
                for i in full_nodes:
                    t += 1
                    if any(b == full_nodes[t-1] for b in active_nodes):
                        full[t-1] = 1.0*np.array(output["DG"][k])[s]
                        s += 1
                    else:
                        full[t-1] = 0
                output["DG"][k] = full
        for k in control_data["control_variables"]["ESS"]:
            s = 0
            t = 0
            if k:
                full = np.zeros(len(full_nodes))
                for i in full_nodes:
                    t += 1
                    if any(b == full_nodes[t-1] for b in active_ESS):
                        full[t-1] = 1.0*np.array(output["ESS"][k])[s]
                        s += 1
                    else:
                        full[t-1] = 0
                output["ESS"][k]=full

        if bool(control_data["MPC_activate"]):
            full_soc = np.zeros(len(full_nodes))
            s = 0
            t = 0
            for i in full_nodes:
                t += 1
                if any(b == full_nodes[t-1] for b in active_ESS):
                    full_soc[t-1] = 1.0*np.array(output["ESS"]["SOC"])[s]
                    s += 1
                else:
                    full_soc[t-1] = 0.0
            output["ESS"]["SOC"]=full_soc


        pv_input_full = np.zeros(len(full_nodes))
        k = 0
        for i in range(len(full_nodes)):
            if any(b == full_nodes[i] for b in active_nodes):
                pv_input_full[i] = pvproduction[k]
                k += 1
            else:
                pv_input_full[i] = 0

        return  output, pv_input_full  


    def resize_out(self, active_nodes,q_sol_centr,p_sol_centr,reactive_power_full,active_power_full,infeasibility_output):

        active_power_sol = np.zeros_like(active_nodes)
        reactive_power_sol = np.zeros_like(active_nodes)
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
                    reactive_power_sol[k] = infeasibility_output[t-1]#reactive_power_full[t-1]
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
                    active_power_sol[k] = active_power_full[t-1]
                    k += 1
                else:
                    pass

        return reactive_power_sol, active_power_sol

    def prioritize(self,q_sol_centr, QMIN,P_activate,n,case):        
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

    def resize_reference(self, control_data, full_nodes, active_nodes, reference, T):
        for k in control_data["control_variables"]["DG"]:
            s = 0
            t = 0
            if k:
                for step in range(T-1):
                    ref_full = np.zeros(len(full_nodes))
                    for i in full_nodes:
                        t += 1
                        if any(b == full_nodes[t-1] for b in active_nodes):
                            ref_full[t-1] = 1.0*np.array(reference["DG"][k][str(step+1)])[s]
                            s += 1
                        else:
                            ref_full[t-1] = 0
                    reference["DG"][k][str(step+1)] = ref_full
        
        return reference