from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt
read_path = "/home/nesc/ldd/msckf_vml/src/msckf_mono/draw/data/vml_1.txt"
save_path = "/home/nesc/ldd/msckf_vml/src/msckf_mono/draw/data/vml_1.png"

data = np.loadtxt(read_path, delimiter=',',skiprows=1)
time = data[:,0]
time = time - time[0]
gt_pos = data[:,1:4]
gt_att = data[:,4:7]
msckf_pos = data[:,8:11]
msckf_att = data[:,11:14]

vml_pos = data[:,15:18]
vml_att = data[:,18:21]

error_msckf = np.sum((msckf_pos-gt_pos) * (msckf_pos-gt_pos),axis=1)
error_msckf = np.sum(error_msckf)/error_msckf.shape[0]
print("msckf",math.sqrt(error_msckf))

error_vml = np.sum((msckf_pos-gt_pos) * (msckf_pos-gt_pos),axis=1)
error_vml = np.sum(error_vml)/error_vml.shape[0]
print("vml",math.sqrt(error_vml))

fig, ax = plt.subplots(2, 3)
for i in range(2):
    for j in range(3):
        if i ==0:
            ax[i][j].plot(time, msckf_pos[:,j], 'r', label="msckf")
            ax[i][j].plot(time, gt_pos[:,j], 'b', label="gt")
            ax[i][j].plot(time, vml_pos[:,j], 'g', label="vml")
        else:
            ax[i][j].plot(time, msckf_att[:,j], 'r', label="msckf")
            ax[i][j].plot(time, gt_att[:,j], 'b', label="gt")
            ax[i][j].plot(time, vml_att[:,j], 'g', label="vml")

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

plt.savefig(save_path, dpi=300)
plt.show()




