%{
===========================================================================
run_simulation.m
DESCRIPTION  :
Program used simulate a quadcopter system being controlled by an LQR-I
discrete-time controller to track a trajectory in the refTraj.csv file.
AUTHOR(S)    : Juan Paredes
EMAIL(S)     : jparedes@umich.edu, juan.augusto.paredes@gmail.com
DATE CREATED : Feb. 6, 2019
LAST REVISED : Jul. 25, 2019
INPUTS
    Quadcopter Model and Controller Parameters, Reference Trajectory .csv file.
OUTPUTS
    Quadcopter states, controller inputs, PWM commands, motor rotation 
rates and time vectors.
===========================================================================
%}

clc
clear all
close all

%% Simulation Parameters
Ts = 0.0001;                    % Simulation time step
Ts_ct = 0.005;                  % Controller sampling time (Default: 200 Hz).
Tfin = 72.5;                    % Simulation total time

%% Quadcopter Model Parameters
x0 = zeros(12,1);               % Initial state
g = 9.80665;                    % Acceleration due to gravity (m/s^2)
m = 1.062;                      % Quadcopter mass (kg)
Jxx = 0.010733986544371;        % Quadcopter inertia along roll axis (kg m^2) 
Jyy = 0.011156873328260;        % Quadcopter inertia along pitch axis (kg m^2) 
Jzz = 0.022900774530029;        % Quadcopter inertia along yaw axis (kg m^2) 
Cr = 9.957365275903740e+03;     % Motor throttle gain (rad/s)
omega_b = 12.351763347436565;   % Motor throttle bias (rad/s)
Tm = 0.245217435487778;         % Time constant of propulsor first-order dynamics (s)
prop_dist = 0.170;              % Distance of each rotor to the geometric center of the quadcopter (m)
CT = 5.724165806708346e-08;     % Rotor thrust coefficient (kg m)
CM = 8.881631641761697e-10;     % Rotor moment coefficient (kg m^2)
phi_sat = [0.8, -0.8];          % Maximum and minimum roll angle values (rad)
theta_sat = [0.8, -0.8];        % Maximum and minimum pitch angle values(rad)
f_sat = [22.7583, 3.4933e-05];  % Maximum and minimum thrust(N)
taux_sat = [1.3679, -1.3679];   % Maximum and minimum roll axis moment(N m)
tauy_sat = [1.3679, -1.3679];   % Maximum and minimum pitch axis moment(N m)
tauz_sat = [0.35, -0.35];       % Maximum and minimum yaw axis moment(N m)

Th_init = min(max((sqrt(m*g/(4*CT)) - omega_b)/Cr,0),1);
Th_init_vec = Th_init*ones(4,1);% Initial set of motor throttles for initial hover.

%% LQR-I Controller Parameters

Kxx = [-0.4638; -0.5333; -0.1560];  % X position controller gains
Kyy = [0.4638; 0.5333; 0.1560];     % Y position controller gains
Kzz = [-10.2854; -6.5162; -1.4903]; % Z position controller gains
KPhi = [0.6433; 0.3714];            % Roll controller gains
KTheta = [0.6455; 0.3734];          % Pitch controller gains
KPsi = [0.1746; 0.1500; 0.0984];    % Yaw controller gains
tauI = 0.5;                         % Integrator anti windup parameter


%% Run simulation

options = simset('SrcWorkspace','current');
out = sim('Quad_Sim_LQR_I',[],options);

%% Save results

save('quad_sim_data.mat', 'Time', 'States', 'Reference', 'Controller_Input', 'System_Input', 'PWM_commands', 'Ts_ct');