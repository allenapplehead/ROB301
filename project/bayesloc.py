import numpy as np
import matplotlib.pyplot as plt

# Actions and measurements
uk = [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, None]
zk = [None, 'o', 'y', 'g', 'b', 'n', 'g', 'b', 'g', 'o', 'y', 'g', 'b']
map = ["y", "g", "b", "o", "o", "g", "b", "o", "y", "g", "b"] # shifted down by 2


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
color_to_idx = {"b":0, "g":1, "y": 2, "o":3, "n":4}
# Simulate the system
states = []
estimates = []
# zk = [x if x else "n" for x in zk]
estimates.append(np.full(11,1/11)) # Initial state
for i in range(len(uk)):
    if False:
        # apply u=1 at the beginning
        pass
    else:
        # state prediction
        #p(xk+1|z0:k) = sigma xk p(xk+1|xk, uk)p(xk|z0:k)
        if uk[i] == None:
            break
        command = uk[i] + 1 # -1 0 1 -> 0 1 2
        state_preds = np.zeros(len(map))
        for j in range(len(map)): # for every xk+1
            s = 0.0
            for k in range(len(map)): #every xk
                #p(xk+1|xk, uk)
                if j - k == 1 or j == 0 and k == 10:
                    s += state_model[command][2] * estimates[-1][k]
                elif k - j == 1 or k == 0 and j == 10:
                    s += state_model[command][0] * estimates[-1][k]
                elif j == k:
                    s += state_model[command][1] * estimates[-1][k]
            state_preds[j] = s
        
        # state update
        #p(xk+1|z0:k+1) = p(zk+1|xk+1)p(xk+1|z0:k)
        state_update = np.zeros(len(map))
        for j in range(len(map)):
            state_update[j] = meas_model[color_to_idx[zk[i+1]]][color_to_idx[map[j]]] * state_preds[j]
        # Normalize
        estimates.append(state_update/np.sum(state_update))
        print(estimates[-1])
    plt.bar(np.arange(2, 13), estimates[-1])
    plt.show()
print(estimates)

# # Plot the states
# plt.figure(1)
# plt.plot(states)
# plt.title('State')
# plt.xlabel('Time')
# plt.ylabel('State')
# plt.show()

