import matplotlib.pyplot as plt
import numpy as np

from os import listdir
from os.path import isfile, join

print("Processing data...")

dataFiles = [f for f in listdir(".") if f.endswith(".txt")]

data = list()

for dataFile in dataFiles:
	file = open(dataFile)
	lines = file.readlines()
	times = list();
	for line in lines:
		times.append(int(line.split(" ")[2]))
	data.append(times)

print("Data loaded")

maxTimes = list()
reachedTargets = list()

print("Collecting statistics...")

for times in data:
	maxTimes.append(np.amax(times))
	reached = 0
	for time in times:
		if time > 0:
			reached = reached + 1
	reachedTargets.append(reached)

print("Generating images...")

plt.figure()
plt.boxplot(maxTimes)
plt.title("Total running time till system completes or gives up")
plt.ylabel("Running time in ms")
plt.savefig("LastGoalReachTime.png")
plt.figure()
plt.boxplot(reachedTargets)
plt.title("Number of cleaning tasks that were completed")
plt.ylabel("Completed cleaning tasks")
plt.savefig("ReachedTargets.png")

print("All done")