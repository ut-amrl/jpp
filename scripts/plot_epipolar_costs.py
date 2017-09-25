import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import sys
import math

def normalize_costs(costs):
  c = np.array(costs)
  norm_costs = c / np.max(c)
  norm_costs[norm_costs < 1e-9] = 1.0
  return norm_costs


costs_file = open(sys.argv[1], "r")
lines = costs_file.readlines()

ndisp = 0
count = 0
costs = []
gt_disp = []

for line in lines:
  parts = line.split(" ")
  if count == 0:
    ndisp = int(parts[0])
  else:
    c = []
    for i in range(ndisp+1):
      c.append(float(parts[i]))
    costs.append(c)
    gt_disp.append(int(parts[ndisp+1]))
  count = count + 1

plt.figure(figsize=(9,6))
plt.xlim(-20, 20)

for i in range(count-1):
  x = [(k-gt_disp[i]) for k in range(ndisp+1)]
  plt.plot(x, costs[i], color='blue', alpha=0.1)
  if i > 2000:
    break


plt.xlabel('Disparity (Ground truth at x = 0)')
plt.ylabel('Normalized Cost')
plt.tight_layout()
plt.draw()
plt.pause(1)
raw_input()