import numpy as np
import matplotlib.pyplot as plt

time = []
data = []

with open("/home/maxseiner/projects/naoBasicKinematics/testTiming1.csv") as f:
    for line in f:
        dat = line.split(",")
        time.append(float(dat[0]))
        data.append(float(dat[1]))
lastTime = time[len(time)-1]
print(lastTime)
with open("/home/maxseiner/projects/naoBasicKinematics/testTiming2.csv") as f:
    for line in f:
        dat = line.split(",")
        time.append(float(dat[0])+lastTime)
        data.append(float(dat[1]))
lastTime2 = time[len(time)-1]
print(lastTime2)
"""with open("/home/alex/Documents/ECE498/naoBasicKinematics/testTiming3.csv") as f:
    for line in f:
        dat = line.split(",")
        time.append(float(dat[0])+lastTime2)
        data.append(float(dat[1]))
"""
plt.figure()
plt.title('battery% vs time(sec)')
plt.plot(time,data)
#plt.plot(range(len(altData)),altData)
plt.show()
