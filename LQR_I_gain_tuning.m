%{
===========================================================================
LQR_I_gain_tuning.m
DESCRIPTION  :
Program used to tune LQR gains via the linearized, decoupled model for
quadcopter states x, xDot, y, yDot, z, zDot, phi, phiDot, theta, thetaDot,
psi, psiDot, implementing integrators on the outer loop.
AUTHOR(S)    : Juan Paredes
EMAIL(S)     : jparedes@umich.edu, juan.augusto.paredes@gmail.com
DATE CREATED : Feb. 6, 2019
LAST REVISED : Jul. 25, 2019
INPUTS
    Quadcopter Mass and Inertia Matrix diagonal elements.
OUTPUTS
    LQR gains for feedback controller.
===========================================================================
%}

clc
clear all
close all


%Q matrix diagonal terms for LQR tuning
%States: x, xDot, xInt, y, yDot, yInt, z, zDot, zInt, phi, phiDot, theta,
% thetaDot, psi, psiDot, psiInt
Q = diag([50, 195, 25, 50, 195, 25, 490, 117, 12.6, 5, 1.5, 5, 1.5, 1, 15, 10]);

%R matrix diagonal terms for LQR tuning
%Inputs:  Desired pitch, Desired roll, Thrust, Torque along roll axis,
% Torque along pitch axis, Torque along yaw axis
R = diag([1000, 1000, 5.5, 10, 10, 1000]);

%Controller update period (1/Hz)
Ts_ct = 0.005;
m = 1.062;                      % Quadcopter mass (kg)
g = 9.80665;
Jxx = 0.010733986544371;        % Quadcopter inertia along roll axis (kg m^2) 
Jyy = 0.011156873328260;        % Quadcopter inertia along pitch axis (kg m^2) 
Jzz = 0.022900774530029;        % Quadcopter inertia along yaw axis (kg m^2) 

%State space model 1: [x; xDot; xInt]
Axx = [0 1 0; 0 0 0; 1 0 0];
Bxx = [0; -g; 0];
Cxx = [1 0 0; 0 0 1];
Dxx = [0; 0];

%State space model 2: [y; yDot; yInt]
Ayy = [0 1 0; 0 0 0; 1 0 0];
Byy = [0; g; 0];
Cyy = [1 0 0; 0 0 1];
Dyy = [0; 0];

%State space model 3: [z ;zDot; zInt]
Azz = [0 1 0; 0 0 0; 1 0 0];
Bzz = [0; -1/m; 0];
Czz = [1 0 0; 0 0 1];
Dzz = [0; 0];

%State space model 4: [phi; phiDot]
Aphi = [0 1; 0 0];
Bphi = [0; 1/Jxx];
Cphi = [1 0];
Dphi = 0;

%State space model 5: [theta; thetaDot]
Atheta = [0 1; 0 0];
Btheta = [0; 1/Jyy];
Ctheta = [1 0];
Dtheta = 0;

%State space model 6: [psi ;psiDot; psiInt]
Apsi = [0 1 0; 0 0 0; 1 0 0];
Bpsi = [0; 1/Jzz; 0];
Cpsi = [1 0 0; 0 0 1];
Dpsi = [0; 0];

%State space model for whole attitude
Ablock = blkdiag(Axx, Ayy, Azz, Aphi, Atheta, Apsi);
Bblock = blkdiag(Bxx, Byy, Bzz, Bphi, Btheta, Bpsi);
Cblock = blkdiag(Cxx, Cyy, Czz, Cphi, Ctheta, Cpsi);
Dblock = blkdiag(Dxx, Dyy, Dzz, Dphi, Dtheta, Dpsi);

%Continuous state space system
sysCT = ss(Ablock, Bblock, Cblock, Dblock);

%Discrete state space system
sysDT = c2d(sysCT, Ts_ct, 'zoh');

Ad = sysDT.A;
Bd = sysDT.B;

%Discrete LQR gain calculation
K = dlqr(Ad, Bd, Q, R);
%K = diag([Kxx, Kyy, Kzz, Kphi, Ktheta, KPsi]);

Kxx =       K(1,1:3);
Kyy =       K(2,4:6);
Kzz =       K(3,7:9);
Kphi =      K(4,10:11);
Ktheta =    K(5,12:13);
Kpsi =      K(6,14:16);