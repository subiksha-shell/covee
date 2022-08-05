import csv
import os
import numpy as np

class save_schedule:

    def __init__(self):
        pass

  
    def save_csv(self, output):
        cwd = os.getcwd()
        wd = os.path.join(cwd, 'covee/control_strategies/scheduler/schedule')
        wd.replace("\\", "/")

        if output["DG"]["reactive_power"] is not None:  # tbc
            rows = np.transpose(output["DG"]["reactive_power"])
            with open(os.path.join(wd, 'reactive_power.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()

        if output["DG"]["active_power"] is not None:  # tbc
            rows = np.transpose(output["DG"]["active_power"])
            with open(os.path.join(wd, 'active_power.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()

        if output["ESS"]["active_power"] is not None:
            rows = np.transpose(output["ESS"]["active_power"])
            with open(os.path.join(wd, 'active_power_batt.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()

            rows = np.transpose(output["ESS"]["SOC"])
            with open(os.path.join(wd, 'SoC.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
                wr = csv.writer(csv_file)
                for row in rows:
                    wr.writerow(row)
            csv_file.close()               
        
        rows = np.transpose(output["voltage"]["controlled"])
        with open(os.path.join(wd, 'voltage.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
            wr = csv.writer(csv_file)
            for row in rows:
                wr.writerow(row)
        csv_file.close()

        rows = np.transpose(output["voltage"]["uncontrolled"])
        with open(os.path.join(wd, 'voltage_uncontrolled.csv'), 'w+', encoding="ISO-8859-1", newline='') as csv_file:
            wr = csv.writer(csv_file)
            for row in rows:
                wr.writerow(row)
        csv_file.close()