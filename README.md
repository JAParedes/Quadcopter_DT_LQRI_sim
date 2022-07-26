# Quadcopter control using discrete-time LQR-I
Matlab files to perform a simulation in which a quadcopter system is controlled by a LQR-I discrete-time controller to track a trajectory. The controller features and inner-outer loop architecture. These files were used to obtain the results shown in [1].

## Files
* **Quad_Sim_LQR_I.m** : Simulink file that sets up the quadcopter continuous-time model and the LQR-I discrete-time controller. .
* **run_simulation.m** : Sets up the model and controller parameters and runs the simulation in **Quad_Sim_LQR_I.m**.
* **plot_simulation_results.m** : Plots simulations results obtained by **run_simulation.m**.
* **LQR_I_gain_tuning.m** : Helps tune LQR controllers based on quadcopter physical properties. These can be then set in **run_simulation.m**.
* **trajectory_generator.m** : Generates an inclined circular trajectory .csv file that can be used for **Quad_Sim_LQR_I.m**.
* **trapzFilter.m** : Second-order discrete-time filter used in **trajectory_generator.m**.
* **ref_Traj.csv** : Reference trajectory .csv file that can be used to test **run_simulation.m**.

## Citing work

* **[1] Paredes, J., Sharma, P., Ha, B., Lanchares, M., Atkins, E., Gaskell, P., & Kolmanovsky, I.**  (2021). Development, implementation, and experimental outdoor evaluation of quadcopter controllers for computationally limited embedded systems. Annual Reviews in Control, 52, 372-389.
