import numpy as np

class optimization_data():

    def __init__(self,scheduler_data,forecast, grid_data, matrixes):
        self.scheduler_data = scheduler_data
        self.forecast = forecast
        self.grid_data = grid_data
        self.matrixes = matrixes
        self.ref = scheduler_data["VMAX"]-scheduler_data["VMIN"]
        self.We0 = self.ref/(scheduler_data["P_DG_MAX"]-scheduler_data["P_DG_MIN"]+1e-5)*2 #power weight (active and reactive)
        self.Ws0 = [0] * self.grid_data["nb"]
        self.SoCmid0 = [0] * self.grid_data["nb"]
        # for i in battery_data['nodes_with_battery']:
        #     self.Ws0[i] = self.ref/(battery_data['SoCmax'][i]-battery_data['SoCmin'][i])   #State of Charge weight, based on the actual values
        #     self.SoCmid0[i] = (battery_data['SoCmax'][i] +battery_data['SoCmin'][i])/2 #desired SoC        
        
    def weights(self): # For now basic weights calculation
        weights = {}
        weights.update({"V": [1] * self.matrixes["n_tot"]})
        weights.update({"P_DG": [1] * self.matrixes["n_tot"]})
        weights.update({"Q_DG": [1] * self.matrixes["n_tot"]})
        weights.update({"P_ESS": [1] * self.matrixes["n_tot"]})

        return  weights

    