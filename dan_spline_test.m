clear all
close all

SetupBipedRobot2
addpath(strcat(pwd(),'/preview_control'))
addpath(strcat(pwd(),'/graphing'))
walk_config

% calculate the preview LQR gains
% defining the system
T = .005;
t = 0:T:100000;
nTimeSteps = 1.6/T; % preview length
steps = 1:nTimeSteps;
zc = uLINK(BODY).p(3) % height of the CoM above the ground
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

xsteps = [ 0 .3 .6  .9 .9 .9 .9];
ysteps = [.1 .1 -.1 .1 -.1  0  0  0];
pxref = footsteps(xsteps,nTimeSteps);
pyref = footsteps(ysteps,nTimeSteps);

RIGHT_FOOT = 0;
LEFT_FOOT = 1;
stance_foot = RIGHT_FOOT;

x_com = [0 0 0]' % initial state for x, x', x"
y_com = [0 0 0]' % initial state for y, y', y"
err0 = 0

% for now, just one step.


xsteps = [0 0];
ysteps = [0 .1];
zsteps = [0 .2 0];
N = 320;

t1 = linspace(0,N/2,20);
t2 = linspace(N/2,N,20);

% Swing foot trajectory:
% ztrajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,zsteps(1),zsteps(2),0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,zsteps(2),zsteps(3),0,0);
zsol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol,...
               a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% x trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,xsteps(1),0,0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,0,xsteps(2),0,0);
xsol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% y trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,ysteps(1),0,0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,0,ysteps(2),0,0);
ysol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% simulate desired COM trajectory
[ x_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pxref,x_com,err0,nTimeSteps);
[ y_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pyref,y_com,err0,nTimeSteps);

op = uLINK(RLEG_J5).p

% Write joint angles to robot
figure('units','normalized','position',[.1 .1 .8 .8])
for i=1:length(xsol)
    % set body
    uLINK(BODY).p = [x_com_ref(2,i), y_com_ref(2,i), 0.7]';
    uLINK(BODY).R = eye(3); 
    
    % set right foot
    Rfoot.p = [op(1),op(2),0]';
    Rfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
    
    % set left foot
    Lfoot.p = [xsol(i)-x_com_ref(2,i),ysol(i)-y_com_ref(2,i),zsol(i)]';
    Lfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR3 = IK_leg(uLINK(BODY),0.1,0.3,0.3,Lfoot);
    for n=0:5
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qR3(n+1);
    end
    ForwardKinematics(1);
    clf
    DrawRobot
    view(60,40)
    fprintf('Type any key for another pose, Ctrl-C to abort\n');
    pause(.05)
    
end

% Swing foot trajectory:

xsteps = [0 0];
ysteps = [.1 0];
zsteps = [0 .2 0];

% ztrajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,zsteps(1),zsteps(2),0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,zsteps(2),zsteps(3),0,0);
zsol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% x trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,xsteps(1),0,0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,0,xsteps(2),0,0);
xsol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% y trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,N/2,ysteps(1),0,0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(N/2+1,N,0,ysteps(2),0,0);
ysol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);

% simulate desired COM trajectory
x_com = [uLINK(BODY).p(1) 0 0]'
y_com = [uLINK(BODY).p(2) 0 0]'

[ x_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pxref(nTimeSteps:end),x_com,err0,nTimeSteps); % TODO: Figure out how to translate the state to the next time step
[ y_com_ref,~,~ ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,pyref(nTimeSteps:end),y_com,err0,nTimeSteps);

op = uLINK(LLEG_J5).p

% Write joint angles to robot
figure('units','normalized','position',[.1 .1 .8 .8])
for i=1:length(xsol)
    % set body
    uLINK(BODY).p = [x_com_ref(2,i), y_com_ref(2,i), 0.7]';
    uLINK(BODY).R = eye(3); 
    
    % set right foot
    Rfoot.p = [xsol(i)-x_com_ref(2,i),ysol(i)-y_com_ref(2,i),zsol(i)]'; 
    Rfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
    
    % set left foot
    Lfoot.p = [op(1),op(2),0]';
    Lfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR3 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Lfoot);
    for n=0:5
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qR3(n+1);
    end
    ForwardKinematics(1);
    clf
    DrawRobot
    view(60,40)
    fprintf('Type any key for another pose, Ctrl-C to abort\n');
    pause(.05)
    
end
