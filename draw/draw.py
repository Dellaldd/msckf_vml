from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt
file_name = "test.txt"
save_name = "test.png"
read_path = "draw/data/" + file_name
data = np.loadtxt(read_path, delimiter=',',skiprows=1)
time = data[:,0]
msckf_pos = data[:,1:4]
gt_pos = data[:,4:7]
msckf_att = data[:,13:17]
gt_att = data[:,13:17]
error_pos = np.sum((msckf_pos-gt_pos) * (msckf_pos-gt_pos),axis=1)
error_pos = np.sqrt(error_pos)
print(error_pos.shape[0])
print(np.sum(error_pos)/error_pos.shape[0])
fig, ax = plt.subplots(2, 3)
for i in range(2):
    for j in range(3):
        if i ==0:
            ax[i][j].plot(time, msckf_pos[:,j], 'r*', label="msckf")
            ax[i][j].plot(time, gt_pos[:,j], 'b*', label="gt")
        else:
            ax[i][j].plot(time, msckf_att[:,j], 'r*', label="msckf")
            ax[i][j].plot(time, gt_att[:,j], 'b*', label="gt")

# for i in range(3):
#     for j in range(3):
#         ax[i+1][j].plot(data[:,0], data[:,i*6+j+7], 'r*', label = "local")  # 7  8  9     13 14 15   19 20 21
#         ax[i+1][j].plot(data[:,0], data[:,i*6+j+10], 'b-', label = "target") # 10 11 12    16 17 18   22 23 24

ax[0, 0].set_title("position x")
ax[0, 1].set_title("position y")
ax[0, 2].set_title("position z")
ax[1, 0].set_title("attitude x")
ax[1, 1].set_title("attitude y")
ax[1, 2].set_title("attitude z")

lines, labels = fig.axes[-1].get_legend_handles_labels()
fig.legend(lines, labels)
fig.tight_layout()
save_path = "draw/data/" + save_name
plt.savefig(save_path, dpi=300)
plt.show()




