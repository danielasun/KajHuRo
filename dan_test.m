% Hacking around to use Kajita's code

clear all 
close all

% Saving until you're able to use this later on
% addpath(strcat(pwd,'/VREP_matlab_remoteAPI'))
% vrep=remApi('remoteApi');
% clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
addpath(strcat(pwd(),'/preview_control'))

% % defining the system
% T = .005;
% t = 0:T:100000;
% N = 3.2/T; % preview length
% steps = 1:N;
% zc = .814;
% gconst = 9.8; % m/s 
% 
% A = [1 T T^2;
%      0 1 T;
%      0 0 1];
%  
% B = [T^3/6; T^2/2; T];
% 
% C = [ 1 0 -zc/gconst];
% 
% D = 0;
% 
% n = size(A,1); % dimension of state vector
% p = size(C,1); % dimension of output vector
% r = size(B,2); % dimension of input vector
% 
% Qe = 1;
% Qx = diag([0 0 0]);
% R = 1e-6;
% 
% ST = .8/T % time steps between each step
% xsteps = [0 0 0 .3 .6 .9 .9 .9 .9];
% ysteps = [0 0 .1 -.1 .1 -.1 0 0 0];
% pxref = footsteps(xsteps,ST);
% pyref = footsteps(ysteps,ST);
% 
% % xstep = .3;
% % ystep = .1;
% % chu = ones(1,.8/T); % chunks of walking so that it's easy for me to make the trajectory
% % 
% % pxref = [0*chu 0*chu 0*chu 1*xstep*chu 2*xstep*chu 3*xstep*chu 3*xstep*chu 3*xstep*chu 3*xstep*chu ];
% % pyref = [0*chu 0*chu ystep*chu -ystep*chu ystep*chu -ystep*chu 0*chu 0*chu 0*chu];
% 
% sys = ss(A,B,C,D,T)
% 
% [sys_t,Gi,Gx,Gd]= preview_control(sys,N,Qe,Qx,R)
% 
% At = sys_t.a
% Bt = sys_t.b
% Ct = sys_t.c

%

SetupBipedRobot2
DrawRobot

% MoveJoints([4,5,6],[-30,60,-30]*ToRad)
ForwardKinematics(1)

CoM = calcCoM;
DrawBall(CoM,.025)

% Rfoot.p = [0, -0.1, 0]' + 0.2*(rand(3,1)-0.5);
% Rfoot.R = RPY2R(1/2*pi*(rand(3,1)-0.5));  %  -pi/4 < q < pi/4
% qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
% 
% for n=0:5
%     uLINK(RLEG_J0+n).q = qR2(n+1);
% end

ForwardKinematics(1);
clf
DrawRobot;

t = linspace(0,2*pi,30);
circle_radius = .2;
xcirc = circle_radius*cos(t);
ycirc = circle_radius*sin(t);
zcirc = .3*cos(2*t)
plot(xcirc,ycirc)

for i=1:length(xcirc)
    Rfoot.p = [xcirc(i), -ycirc(i), zcirc(i)]';
    Rfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
    Lfoot.p = [xcirc(i), ycirc(i), zcirc(i)]';
    Lfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
    qR3 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Lfoot);
    for n=0:5
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qR3(n+1);
    end
    ForwardKinematics(1);
    clf
    DrawRobot
    fprintf('Type any key for another pose, Ctrl-C to abort\n');
    pause(.05)
end
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
