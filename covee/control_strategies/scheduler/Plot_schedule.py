import csv
import os
import numpy as np
import matplotlib.pyplot as plt


V_base = 1
P_base = 10e3

plt.rcParams["font.family"] = "serif"
plt.rcParams["figure.figsize"] = (15,7.5)
plt.rc('legend', fontsize=20, loc='upper right')    # legend fontsize

cwd = os.getcwd()

wd = os.path.join(cwd, 'covee/control_strategies/scheduler/schedule')
wd.replace('\\', '/')

wd1 = os.path.join(cwd, 'covee/control_strategies/scheduler/schedule/plots')
wd1.replace('\\', '/')

t = {}
v = {}
v_not = {}

# Voltage 
# ============================================================
with open(os.path.join(wd, 'voltage.csv')) as csv_file:
    distributed = csv.reader(csv_file, delimiter=',')
    x = list(distributed)
    distributed = np.array(x).astype("float")*V_base

with open(os.path.join(wd, 'voltage_uncontrolled.csv')) as csv_file:
    v_not_distributed = csv.reader(csv_file, delimiter=',')
    x = list(v_not_distributed)
    v_not_distributed = np.array(x).astype("float")*V_base

lenght = distributed.shape[1]-1
for r in range(lenght):
    t.update({"t"+str(r): np.matrix(distributed)[:,lenght]})
    v.update({"node_"+str(r): np.matrix(distributed)[:,r]})
    v_not.update({"node_"+str(r): np.matrix(v_not_distributed)[:,r]})

min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])
limitMAX = np.array([1.0495]*min_leght)
limitMIN = np.array([0.95]*min_leght)

plt.figure(1)
plt.plot(limitMAX,c = "khaki",linewidth=8, label="V_MAX")
plt.plot(limitMIN,c = "khaki",linewidth=8, label="V_MIN")
for r in range(lenght):
    plt.plot(v_not["node_" + str(r)][0:min_leght],linewidth=2, c = "lightgray")
    plt.plot( v["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "dimgray")
plt.plot(v_not["node_" + str(r-1)][0:min_leght],linewidth=2, c = "lightgray", label="V"+" Not Controlled")
plt.plot(v["node_" + str(r-1)][0:min_leght],linewidth=2, marker="*", c = "dimgray", label="V")
plt.plot(v["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "indigo", label="V"+" ["+str(r+1)+"]")
plt.plot(v_not["node_" + str(r)][0:min_leght],linewidth=2, c = "turquoise", label="V"+" ["+str(r+1)+"]"+" Not Controlled")
axes = plt.gca()
axes.set_ylim([0.85, 1.1])
plt.xlabel("Iterations")
plt.ylabel("Voltage [p.u.]")
plt.legend(facecolor='white', framealpha=1, loc='lower left')
plt.savefig(wd1+'/voltage.eps')
plt.savefig(wd1+'/voltage.png')

t = {}
q = {}
# Reactive Power 
# ============================================================
with open(os.path.join(wd, 'reactive_power.csv')) as csv_file:
    reactive_power = csv.reader(csv_file, delimiter=',')
    x = list(reactive_power)
    reactive_power = np.array(x).astype("float")*V_base


lenght = reactive_power.shape[1]-1
for r in range(lenght):
    t.update({"t"+str(r): np.matrix(reactive_power)[:,lenght]})
    q.update({"node_"+str(r): np.matrix(reactive_power)[:,r]})

min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])
# limitMAX = np.array([1.0495]*min_leght)
# limitMIN = np.array([0.9]*min_leght)
# limitMAX_20 = np.array([1.07]*min_leght)

plt.figure(2)
for r in range(lenght):
    plt.plot( q["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "dimgray")
plt.plot(q["node_" + str(r-1)][0:min_leght],linewidth=2, marker="*", c = "dimgray", label="Q DG")
plt.plot(q["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "indigo", label="Q DG"+" ["+str(r+1)+"]")
axes = plt.gca()
# axes.set_ylim([0.85, 1.1])
plt.xlabel("Iterations")
plt.ylabel("Reactive Power [p.u.]")
plt.legend(facecolor='white', framealpha=1, loc='lower left')
plt.savefig(wd1+'/reactive_power.eps')
plt.savefig(wd1+'/reactive_power.png')


t = {}
p = {}
# Active Power 
# ============================================================
with open(os.path.join(wd, 'active_power.csv')) as csv_file:
    active_power = csv.reader(csv_file, delimiter=',')
    x = list(active_power)
    active_power = np.array(x).astype("float")*V_base


lenght = active_power.shape[1]-1
for r in range(lenght):
    t.update({"t"+str(r): np.matrix(active_power)[:,lenght]})
    p.update({"node_"+str(r): np.matrix(active_power)[:,r]})

min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])
# limitMAX = np.array([1.0495]*min_leght)
# limitMIN = np.array([0.9]*min_leght)
# limitMAX_20 = np.array([1.07]*min_leght)

plt.figure(3)
for r in range(lenght):
    plt.plot( p["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "dimgray")
plt.plot(p["node_" + str(r-1)][0:min_leght],linewidth=2, marker="*", c = "dimgray", label="P DG")
plt.plot(p["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "indigo", label="P DG"+" ["+str(r+1)+"]")
axes = plt.gca()
# axes.set_ylim([0.85, 1.1])
plt.xlabel("Iterations")
plt.ylabel("Active Power [p.u.]")
plt.legend(facecolor='white', framealpha=1, loc='lower left')
plt.savefig(wd1+'/active_power.eps')
plt.savefig(wd1+'/active_power.png')

t = {}
p_ESS = {}
# Active Power ESS
# ============================================================
with open(os.path.join(wd, 'active_power_batt.csv')) as csv_file:
    active_power_batt = csv.reader(csv_file, delimiter=',')
    x = list(active_power_batt)
    active_power_batt = np.array(x).astype("float")*V_base


lenght = active_power_batt.shape[1]-1
for r in range(lenght):
    t.update({"t"+str(r): np.matrix(active_power_batt)[:,lenght]})
    p_ESS.update({"node_"+str(r): np.matrix(active_power_batt)[:,r]})

min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])
# limitMAX = np.array([1.0495]*min_leght)
# limitMIN = np.array([0.9]*min_leght)
# limitMAX_20 = np.array([1.07]*min_leght)

plt.figure(4)
for r in range(lenght):
    plt.plot( p_ESS["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "dimgray")
plt.plot(p_ESS["node_" + str(r-1)][0:min_leght],linewidth=2, marker="*", c = "dimgray", label="P DG")
plt.plot(p_ESS["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "indigo", label="P DG"+" ["+str(r+1)+"]")
axes = plt.gca()
# axes.set_ylim([0.85, 1.1])
plt.xlabel("Iterations")
plt.ylabel("Active Power [p.u.]")
plt.legend(facecolor='white', framealpha=1, loc='lower left')
plt.savefig(wd1+'/active_power_batt.eps')
plt.savefig(wd1+'/active_power_batt.png')


t = {}
SOC = {}
# Active Power ESS
# ============================================================
with open(os.path.join(wd, 'SoC.csv')) as csv_file:
    SoC = csv.reader(csv_file, delimiter=',')
    x = list(SoC)
    SoC = np.array(x).astype("float")*V_base


lenght = SoC.shape[1]-1
for r in range(lenght):
    t.update({"t"+str(r): np.matrix(SoC)[:,lenght]})
    SOC.update({"node_"+str(r): np.matrix(SoC)[:,r]})

min_leght = min([len(t["t" + str(r)]) for r in  range(lenght)])
# limitMAX = np.array([1.0495]*min_leght)
# limitMIN = np.array([0.9]*min_leght)
# limitMAX_20 = np.array([1.07]*min_leght)

plt.figure(5)
for r in range(lenght):
    plt.plot( SOC["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "dimgray")
plt.plot(SOC["node_" + str(r-1)][0:min_leght],linewidth=2, marker="*", c = "dimgray", label="P DG")
plt.plot(SOC["node_" + str(r)][0:min_leght],linewidth=2, marker="*", c = "indigo", label="P DG"+" ["+str(r+1)+"]")
axes = plt.gca()
# axes.set_ylim([0.85, 1.1])
plt.xlabel("Iterations")
plt.ylabel("Active Power [p.u.]")
plt.legend(facecolor='white', framealpha=1, loc='lower left')
plt.savefig(wd1+'/SoC.eps')
plt.savefig(wd1+'/SoC.png')