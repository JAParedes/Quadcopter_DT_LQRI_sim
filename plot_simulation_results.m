%{
===========================================================================
plot_simulation_results.m
DESCRIPTION  :
Program used to plot the simulation results from the .mat file obtained in
the run_simulation.m program. Plots the position and attitude versus the
reference values.
AUTHOR(S)    : Juan Paredes
EMAIL(S)     : jparedes@umich.edu, juan.augusto.paredes@gmail.com
DATE CREATED : Feb. 6, 2019
LAST REVISED : Jul. 25, 2019
INPUTS
    Quadcopter simulation results.
OUTPUTS
    Quadcopter simulation plot.
===========================================================================
%}

close all
clear all
clc

%% Load simulation results

load('quad_sim_data.mat')

%% Plot simulation results

x_ref = Reference(:,1);
x = States(:,1);

y_ref = Reference(:,2);
y = States(:,2);

z_ref = Reference(:,3);
z = States(:,3);

phi_ref = Controller_Input(:,1);
phi = States(:,7);

theta_ref = Controller_Input(:,2);
theta = States(:,8);

psi_ref = Reference(:,7);
psi = States(:,9);

textLabel = 12;
textAxis = 12;
f = figure(1);

f.Position = [600   300   980   650];

fx = subplot(2,3,1);

plot(Time,x,'color',[0, 0.4470, 0.7410],'linewidth',2)
hold on
plot(Time,x_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
ylabel('Position (m)','interpreter','latex','fontsize',textAxis)
legend('$x$','$r_{x}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
set(gca, 'xticklabel', []);
xlim([Time(1) Time(end)])

fphi = subplot(2,3,4);

plot(Time,phi,'color',[0.8500, 0.3250, 0.0980],'linewidth',2)
hold on
plot(Time,phi_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
ylabel('Angle (rad)','interpreter','latex','fontsize',textAxis)
xlabel('Time (s)','interpreter','latex','fontsize',textAxis)
legend('$\phi$','$r_{\phi}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
xlim([Time(1) Time(end)])

fy = subplot(2,3,2);

plot(Time,y,'color',[0.9290, 0.6940, 0.1250],'linewidth',2)
hold on
plot(Time,y_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
legend('$y$','$r_{y}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
set(gca, 'xticklabel', []);
xlim([Time(1) Time(end)])

ftheta = subplot(2,3,5);

plot(Time,theta,'color',[0.4940, 0.1840, 0.5560],'linewidth',2)
hold on
plot(Time,theta_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
xlabel('Time (s)','interpreter','latex','fontsize',textAxis)
legend('$\theta$','$r_{\theta}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
xlim([Time(1) Time(end)])

fz = subplot(2,3,3);

plot(Time,z,'color',[0, 0.5, 0],'linewidth',2)
hold on
plot(Time,z_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
legend('$-z$','$-r_{z}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
set(gca, 'xticklabel', []);
xlim([Time(1) Time(end)])

fpsi = subplot(2,3,6);

plot(Time,psi,'color',[0.6350, 0.0780, 0.1840],'linewidth',2)
hold on
plot(Time,psi_ref,'k--','linewidth',2)
hold off
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontSize',textAxis)
xlabel('Time (s)','interpreter','latex','fontsize',textAxis)
legend('$\psi$','$r_{\psi}$','interpreter','latex','fontsize',textLabel,'Location','southeast')
grid on
xlim([Time(1) Time(end)])


%% Plot

fphi.Position = [0.065 0.065 0.27 0.4];
ftheta.Position = [0.38 0.065 0.27 0.4];
fpsi.Position = [0.69 0.065 0.27 0.4];
fx.Position = [0.065 0.55 0.27 0.4];
fy.Position = [0.38 0.55 0.27 0.4];
fz.Position = [0.69 0.55 0.27 0.4];
