import numpy as np
import matplotlib.pyplot as plt

# Given map (address 0 and 1 don't exist)
map = ["y", "g", "b", "o", "o", "g", "b", "o", "y", "g", "b"]

# Actions and measurements
uk = [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, None]
zk = [None, 'o', 'y', 'g', 'b', 'n', 'g', 'b', 'g', 'o', 'y', 'g', 'b']

# state model p(xk+1|xk = X, uk)
state_model = np.array([[0.85, 0.05, 0.05],
               [0.10, 0.90, 0.10],
               [0.05, 0.05, 0.85]])

# measurement model p(zk|xk)
meas_model = np.array([[0.60, 0.20, 0.05, 0.05],
              [0.20, 0.60, 0.05, 0.05],
              [0.05, 0.05, 0.65, 0.20],
              [0.05, 0.05, 0.15, 0.60],
              [0.10, 0.10, 0.10, 0.10]])

# map colours to indices
col_to_idx = {"b": 0, "g": 1, "y": 2, "o": 3, "n": 4}

# Simulate the system
states = [[1/11] * 11] # initialize uniform probability for first state

for i in range(len(uk)):
    # state prediction
    #p(xk+1|z0:k) = sigma xk p(xk+1|xk, uk)p(xk|z0:k)

    cur_u = uk[i] + 1 # current action (index shifted)

    state_pred = np.zeros(len(map)) # initialize state prediction

    for j in range(len(map)): # cur state
        p_xkp1_zk = 0.0
        for k in range(len(map)): # next state
            if abs(k - j) <= 1:
                p_state = state_model[cur_u][k-j+1] # use state model to update
            elif k == 0 and j == 10: # if next state is leftmost and cur state is rightmost
                p_state = state_model[cur_u][2] # robot has to move right
            elif k == 10 and j == 0: # if next state is rightmost and cur state is leftmost
                p_state = state_model[cur_u][0] # robot has to move left
            else:
                p_state = 0.0 # can't teleport over a step greater than 2

            p_xkp1_zk += p_state * states[i-1][k] # prob of transition to next state
        state_pred[j] = p_xkp1_zk

    # state update
    #p(xk+1|z0:k+1) = p(zk+1|xk+1)p(xk+1|z0:k)
    state_update = []
    if zk == None:
        state_update = state_pred # use previous predictions if no measurement info
    else:
        norm = 0
        for j in range(len(state_pred)):
            norm += meas_model[col_to_idx[zk[i]]][col_to_idx[map[j]]] * state_pred[j]

        for j in range(len(state_pred)):
            p_xkp1_zkp1 = (meas_model[col_to_idx[zk[i]]][col_to_idx[map[j]]] * state_pred[j]) / norm
            state_update.append(p_xkp1_zkp1)

    states.append(state_update)

# Plot the states
plt.figure(1)
plt.plot(states)
plt.title('State')
plt.xlabel('Time')
plt.ylabel('State')
plt.show()

# last timestep ends up at 8