%{
===========================================================================
trajectory_generator.m
DESCRIPTION  :
Program used generate a trajectory for the quadcopter simulation. The
output is an inclined circular trajectory written in a .csv file.
AUTHOR(S)    : Juan Paredes
EMAIL(S)     : jparedes@umich.edu, juan.augusto.paredes@gmail.com
DATE CREATED : Feb. 6, 2019
LAST REVISED : Jul. 25, 2019
INPUTS
    Inclined circular trajectory parameters.
OUTPUTS
    Inclined circular trajectory .csv file.
===========================================================================
%}

close all
clear all
clc

%% Trajectory setup for inclined circular trajectory

% States: x, y, z, xdot, ydot, zdot, yaw

r = 5;          % Radius (m)
dh = 1.5;       % Height differential (m)
V = 2;          % Lateral radial velocity (m/s)
tf_idle_init = 2;
tf = 55;        % Finishing time of trajectory (s)
tf_idle = 5;    % Time after end of trajectory for quadcopter to stay in place (s)
dt = 0.005;     % Controller sampling time (s)

omega_f = V/r;  % Angular velocity (rad/s)
tf_true = tf + tf_idle;

eps = 1.2;      % Second-order digital filter coefficients
wn = 1;
filtFun = @(x) trapzFilter(x, eps, wn, dt); %  Second-order digital filter
filtFun(0);     % Filter initialization

omega = 0;
yaw = 0;
theta = 0;
states = zeros(1+floor(tf_true/dt),7);
t = dt.*(0:floor(tf_true/dt));

%% Trajectory generation

for kk = 1:1+floor(tf_true/dt)

    yaw = yaw + omega*dt;
    theta = theta + omega*dt;
    
    x = r*sin(theta);
    y = r*(cos(theta)-1);
    z = -dh/2*(cos(theta)-1);
    
    xd = r*omega*cos(theta);
    yd = -r*omega*sin(theta);
    zd = dh/2*omega*sin(theta);
    
    states(kk,:) = [x, y, z, xd, yd, zd, yaw];
    
    if (t(kk) >= tf_idle_init && t(kk) <= tf)
        omega = filtFun(omega_f);
    else
        omega = filtFun(0);
    end
    
end

%% Save trajectory in .csv

writematrix(states,'refTraj.csv');