import matplotlib.pyplot as plt

if __name__ == "__main__":
    ########## Definitions ##########
    map = ["y", "g", "b", "o", "o", "g", "b", "o", "y", "g", "b"] # shifted down by 2

    actions = [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, None] # not shifted
    msmts = [None, "o", "y", "g", "b", "n", "g", "b", "g", "o", "y", "g", "b"]
    #msmts = [None, "o", "y", "g", "b", "y", "g", "b", "b", "o", "o", "g", "b"]
    real_states = [8, 9, 10, 11, 12, 2, 3, 4, 4, 5, 6, 7, 8] # interpeted manually based on commands

    state_model = [[0.85, 0.1, 0.05],
                   [0.05, 0.9, 0.05],
                   [0.05, 0.1, 0.85]] # index uk first then xk+1
    
    msmt_model = [[0.6, 0.2, 0.05, 0.05, 0.1],
                  [0.2, 0.6, 0.05, 0.05, 0.1],
                  [0.05, 0.05, 0.65, 0.15, 0.1],
                  [0.05, 0.05, 0.2, 0.6, 0.1]] # index xk first then zk
    
    col_2_idx = {"b": 0, "g": 1, "y": 2, "o": 3, "n": 4}
    idx_2_col = {0: "b", 1: "g", 2: "y", 3: "o", 4: "n"}


    ########## Simulation ##########
    # Tracking variables
    p_dists = [[1/11] * 11] # initial probability uniform
    curr_state = max(range(len(p_dists[0])), key=p_dists[0].__getitem__) # estimate state with largest probability
    est_states = [curr_state + 2] # shifted back up by 2

    k = 0
    while actions[k] != None:
        # Vars at time k
        uk = actions[k] + 1 # shifted up 1 to match state model indexing
        xk = curr_state
        zk = msmts[k]

        # State prediction
        preds = []
        for i in range(len(map)):
            p_xkp1_zk = 0
            for j in range(len(map)):
                if abs(j - i) <= 1:
                    p_state = state_model[uk][i-j+1]
                elif j == 0 and i == 10:
                    p_state = state_model[uk][0]
                elif i == 0 and j == 10:
                    p_state = state_model[uk][2]
                else:
                    p_state = 0

                p_xkp1_zk += p_state * p_dists[len(p_dists)-1][j]
            preds.append(p_xkp1_zk)

        # State update
        update = []
        if zk == None:
            update = preds
        else:
            norm = 0
            for i in range(len(preds)):
                norm += msmt_model[col_2_idx[map[i]]][col_2_idx[zk]] * preds[i]

            for i in range(len(preds)):
                p_xkp1_zkp1 = (msmt_model[col_2_idx[map[i]]][col_2_idx[zk]] * preds[i])/norm
                update.append(p_xkp1_zkp1)

        # Store info and predict next state
        curr_state = max(range(len(update)), key=update.__getitem__)
        p_dists.append(update)
        est_states.append(curr_state + 2)

        k += 1

    
    plt.figure()
    plt.plot(list(range(len(actions))), est_states, label="Estimated State")
    plt.plot(list(range(len(actions))), real_states, label="Real State")
    plt.xlabel("Time k")
    plt.ylabel("Location Number")
    plt.legend()
    plt.show()