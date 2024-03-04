# EV-Speed-Energy-Optimization
This repository contains the Electric Vehicle (EV) dataset and Matlab code for speed control and energy efficiency in EVs using Optimization Techniques

# Title of Project:
Multi-Objective optimization of PI Controller Tuning for speed control and energy efficiency in Electric Vehicles (EVs)

# Description:
This MATLAB code implements a multi-objective optimization approach using Particle Swarm Optimization (PSO) to tune Proportional-Integral (PI) controller parameters for electric vehicle (EV) speed control. The code aims to minimize Mean Squared Error (MSE) for speed tracking performance and energy consumption. It includes simulation models for PID controller tuning and optimization using PSO.

# Usage:
1. Run the MATLAB script PID_MO.m to start the optimization process.
2. The script reads input data from Speed_Input.csv, which contains time-speed data for the electric vehicle.
3. Adjust the parameters in the script as needed, such as PSO settings (c, w, particles, iteration) and objective function weights (W_mse, W_energy).
4. The script initializes and runs the PSO algorithm to optimize PI controller parameters (kp, ki) for the given input data.
5. Upon completion, the script generates plots to visualize the optimization process and results.

# Files:
MO_Final.m: Main MATLAB script for PSO-based PID controller tuning.
EV_Data_New.xlsx: Input data file of the EV.
PID_MO.slx: Simulink model for complete setup modeling.
README.md: Documentation file providing instructions and details about the code.

# Requirements:
MATLAB (R2020a or later)
Simulink
