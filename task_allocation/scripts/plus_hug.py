import numpy as np
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment
from scipy.optimize import minimize

from matplotlib import pyplot as plt

np.random.seed(15)
# Number of robots and tasks
n_robot = 5
n_task = 5

# randomly generates the pose of robots and tasks
robot_pos = np.random.uniform(-1,1,size=(n_robot, 3))
task_pos = np.random.uniform(-1,1,size=(n_task, 3))

print(robot_pos)

# Create figure
fig, ax = plt.subplots(1,2,figsize=(10,5))
plt.rc('font', size=16)  

# plot initial mission environment
# ax[0].plot(robot_pos[:,0], robot_pos[:,1], 'k^',label="robot")
# ax[0].plot(task_pos[:,0], task_pos[:,1], 'bo', label="task")
# ax[0].set_title("Mission Environment")
# ax[0].set(xlim=(-1.2,1.2),ylim=(-1.2,1.2))
# ax[0].set_aspect("equal")
# ax[0].legend()

# calculates a distance matrix as the cost
cost = distance_matrix(robot_pos, task_pos)
# Hungarian algorithm
# print(f"cost::{cost}");print()
row_ind, col_ind = linear_sum_assignment(cost)
# cacluates the total cost of the result
hungarian_total_cost = cost[row_ind, col_ind].sum()

print("Hungarian Result: {}".format(col_ind))
print("Hungarian Cost: {:.3}".format(hungarian_total_cost))

# plot Hungarian assignments
for i in range(len(col_ind)):
    ax[0].plot([robot_pos[row_ind,0],task_pos[col_ind,0]],[robot_pos[row_ind,1],task_pos[col_ind,1]],'r--')
ax[0].plot(robot_pos[:,0], robot_pos[:,1], 'k^',label="robot")
ax[0].plot(task_pos[:,0], task_pos[:,1], 'bo', label="task")
ax[0].set_title("Hungarian algorithm cost: {:.3}".format(hungarian_total_cost))
ax[0].set(xlim=(-1.2,1.2),ylim=(-1.2,1.2))
ax[0].set_aspect("equal")
ax[0].legend()


# greedy algorithm
greedy_task = []
greedy_total_cost = 0
for robot in range(n_robot):
    cand_task = list(np.argsort(cost[robot, :]))
    while True:
        task = cand_task[0]
        if task not in greedy_task:
            greedy_task.append(task)
            greedy_total_cost += cost[robot, task]
            break
        else:
            cand_task.pop(0)

greedy_task = np.array(greedy_task)
print("Greedy Result: {}".format(greedy_task))
print("Greedy Cost: {:.3}".format(greedy_total_cost))

# plot Greedy assignments
for i in range(len(col_ind)):
    ax[1].plot([robot_pos[row_ind,0],task_pos[greedy_task,0]],[robot_pos[row_ind,1],task_pos[greedy_task,1]],'r--')
ax[1].plot(robot_pos[:,0], robot_pos[:,1], 'k^',label="robot")
ax[1].plot(task_pos[:,0], task_pos[:,1], 'bo', label="task")
ax[1].set_title("Greedy algorithm cost: {:.3}".format(greedy_total_cost))
ax[1].set(xlim=(-1.2,1.2),ylim=(-1.2,1.2))
ax[1].set_aspect("equal")
ax[1].legend()

fig.tight_layout()

plt.show()

plt.savefig("High_resolution_600_2.png", dpi=600)

