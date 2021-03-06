clc
clear
close all

% profile on

% defining the system
T = .002;
t = 0:T:100000;
N = 1.6/T; % preview length
steps = 1:N;
zc = .7;
gconst = 9.8; % m/s 

A = [1 T T^2;
     0 1 T;
     0 0 1];
 
B = [T^3/6; T^2/2; T];

C = [ 1 0 -zc/gconst];

D = 0;

n = size(A,1); % dimension of state vector
p = size(C,1); % dimension of output vector
r = size(B,2); % dimension of input vector

Qe = 1;
Qx = diag([0 0 0]);
R = 1e-6;

xsteps = [0 0 0 .3 .6 .9 .9 .9 .9];
ysteps = [0 -.1 .1 -.1 .1 -.1 0 0 0];
pxref = footsteps(xsteps,N);
pyref = footsteps(ysteps,N);

sys = ss(A,B,C,D,T)

[sys_t,Gi,Gx,Gd]= preview_control(sys,N,Qe,Qx,R)

At = sys_t.a
Bt = sys_t.b
Ct = sys_t.c

x0 = [0 0 0]';
y0 = [.0 0 0]';
err_x0 = 0; % error in both systems to start out is zero
err_y0 = 0;


[qx,zmp_x,ux] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pxref,x0,err_x0,N);
[qy,zmp_y,uy] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pyref,y0,err_y0,N);

figure('units','normalized','position',[0 0.5 0.35 0.4]);
subplot(2,4,1:4)
hold all
plot(t(1:size(zmp_x,2)),pxref(1:size(zmp_x,2)))
plot(t(1:size(zmp_x,2)),zmp_x)
plot(t(1:size(zmp_x,2)),qx(2,:),'r-')
legend('pxref','zmp','x CoM')
title('x zmp')
xlabel('time [s]')
ylabel('x [m]')
subplot(2,4,5:8)
plot(t(1:length(ux)),ux)
title('control input')

figure('units','normalized','position',[0 0.025 0.35 0.4]);
subplot(2,4,1:4)
hold all
plot(t(1:size(zmp_y,2)),pyref(1:size(zmp_y,2)))
plot(t(1:size(zmp_y,2)),zmp_y)
plot(t(1:size(qy,2)),qy(2,:),'r-')
legend('pyref','zmp','y CoM')
title('y zmp')
xlabel('time [s]')
ylabel('y [m]')
subplot(2,4,5:8)
plot(t(1:length(uy)),uy)
title('control input')


% showing that you can resume in the middle of the run
y0 = qy(2:4,767);
err_y0 = qy(1,767);
pyref = pyref(767:end)

[qy,zmp_y,uy] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pyref,y0,err_y0,N);

figure('units','normalized','position',[0 0.025 0.35 0.4]);
subplot(2,4,1:4)
hold all
plot(t(1:size(zmp_y,2)),pyref(1:size(zmp_y,2)))
plot(t(1:size(zmp_y,2)),zmp_y)
plot(t(1:size(qy,2)),qy(2,:),'r-')
legend('pyref','zmp','y CoM')
title('y zmp')
xlabel('time [s]')
ylabel('y [m]')
subplot(2,4,5:8)
plot(t(1:length(uy)),uy)
title('control input')








