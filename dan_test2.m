clear all
close all

SetupBipedRobot2
addpath(strcat(pwd(),'/preview_control'))
addpath(strcat(pwd(),'/graphing'))

% calculate the preview LQR gains
% defining the system
T = .005;
t = 0:T:100000;
nTimeSteps = 1.6/T; % preview length
steps = 1:nTimeSteps;
zc = .7; % height of the CoM above the ground
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

% calculate the preview control gains and the simulation system
sys = ss(A,B,C,D,T)
[sys_t,Gi,Gx,Gd]= preview_control(sys,nTimeSteps,Qe,Qx,R)

xsteps = [0  0  .3 .6  .9 .9 .9 .9];
ysteps = [0 .1 -.1 .1 -.1  0  0  0];
pxref = footsteps(xsteps,nTimeSteps);
pyref = footsteps(ysteps,nTimeSteps);

RIGHT_FOOT = 0;
LEFT_FOOT = 1;
stance_foot = RIGHT_FOOT;

x_com = [0 0 0]' % initial state for x, x', x"
y_com = [0 0 0]' % initial state for y, y', y"
err0 = 0
% repeatedly:

    % calculate what the next few steps will be (maybe make it start on one
    % particular foot to start out??)
    
    % for each step:
    for step = 1:length(xsteps)
        % calculate the COM trajectory given the desired ZMP pathway 
        % by simulating the system using the preview controller 
        [ x_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pxref,x_com,err0,nTimeSteps);
        [ y_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pyref,y_com,err0,nTimeSteps);
        
        % calculate the swing foot's trajectory using a spline
        

        % for each time step:
            % Given the COM trajectory, have the stance foot follow the inverse
            % kinematics given by the simulated ZMP trajectory
            
            % have the other foot lift slightly off the ground using splines. Use configuration
            % variables for this. Approximate the COM's path using the pelvis link.
        
        % swap the stance and floating feet
        
        
    end
    