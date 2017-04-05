% testing speed of algorithm

clc
clear
close all

profile on

% defining the system
T = .005;
t = 0:T:100000;
N = 3.2/T; % preview length
steps = 1:N;
zc = .814;
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

ST = .8/T % time steps between each step
xsteps = [0 0 0 .3 .6 .9 .9 .9 .9];
ysteps = [0 0 .1 -.1 .1 -.1 0 0 0];
pxref = footsteps(xsteps,ST);
pyref = footsteps(ysteps,ST);

sys = ss(A,B,C,D,T)

[sys_t,Gi,Gx,Gd]= preview_control(sys,N,Qe,Qx,R)

At = sys_t.a
Bt = sys_t.b
Ct = sys_t.c

%% simulating x0
x0 = [0 0 0]';
q = [zeros(1,p), x0']';

G = [Gi Gx];

u = [];
y = Ct*q;
It = [eye(p);zeros(size(B))]

for i=1:length(pxref)-N-1
    u(end+1) = -G*q(:,i) +  Gd*pxref(i:i+N-1)';
    q(:,end+1) = At*q(:,i) + Bt*u(:,i) - It*pxref(i+1);
    y(:,end+1) = C*q(p+1:end,i);
end

%% simulating y0

x0 = [0 0 0]'; % still use the same initial conditions
q = [zeros(1,p),x0']';

u = [];
y = Ct*q;

for i=1:length(pyref)-N-1
    u(end+1) = -G*q(:,i) +  Gd*pyref(i:i+N-1)';
    q(:,end+1) = At*q(:,i) + Bt*u(:,i) - It*pyref(i+1);
    y(:,end+1) = C*q(p+1:end,i);
end

profile off
prof_data = profile('info')
profile viewer