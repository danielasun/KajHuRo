% Preview control of zmp

clc
clear
close all

% defining the system
T = .005;
t = 0:T:100000;
N = 1.6/T; % preview length
steps = 1:N;
zc = .814;
gconst = 9.8; % m/s 

A = [1 T T^2;
     0 1 T;
     0 0 1];
 
B = [T^3/6; T^2/2; T];

C = [ 1 0 -zc/gconst];

Qe = 1;
Qx = diag([0 0 0]);
R = 1e-6;

xstep = .3;
ystep = .1;
chu = ones(1,.8/T); % chunks of walking so that it's easy for me to make the trajectory

% pxref = zeros(1,10*N);
pxref = [0*chu 0*chu 0*chu 1*xstep*chu 2*xstep*chu 3*xstep*chu 3*xstep*chu 3*xstep*chu 3*xstep*chu ];
pyref = [0*chu 0*chu ystep*chu -ystep*chu ystep*chu -ystep*chu 0*chu 0*chu 0*chu];

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Preview Control Starts here  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = size(A,1); % dimension of state vector
p = size(C,1); % dimension of output vector
r = size(B,2); % dimension of input vector

if rank(ctrb(A,B)) ~= size(A,1)
    disp 'System is not controllable, cannot proceed'
    return
end

if rank(obsv(A,C)) ~= size(A,1)
    disp 'System is not controllable, cannot proceed'
    return
end

At = [1 C*A; zeros(n,1) A];
Bt = [C*B; B];
Ct = [1 0 0 0];
It = [eye(p); zeros(size(A,1),p)];
Ft = [C*A; A];

Qt = blkdiag(Qe,Qx);
Ad = [zeros(p*N-1,1) eye(p*N-1,p*N-1); zeros(1,p*N)];
Vt = [-It, zeros(p+n,(p*N-p))];

% Final system formulation
Abar = [At, Vt; zeros(p*N,p+n) Ad]; 
Bbar = [Bt; zeros(p*N,r)];
Qbar = blkdiag(Qt,zeros(p*N));

% Solving for the LQR gains
[KKKK,Pbar,e] = dlqr(Abar,Bbar,Qbar,R,0);
Pt = Pbar(1:p+n,1:p+n);
Xt = Pbar(1:p+n,p+n+1:end);
Ktilde = inv(R+Bt'*Pt*Bt)*Bt'*Pt*At;
At_cl = At-Bt*inv(R+Bt'*Pt*Bt)*Bt'*Pt*At;

Xtt(:,1) = -At_cl'*Pt*It;
for i=2:N
    Xtt(:,i) = At_cl'*Xtt(:,i-1);
    Gdd(i) = inv(R + Bt'*Pt*Bt)*Bt'*Xtt(:,i-1);
end

Zt = Pbar(p+n+1:end,p+n+1:end);

Gi = inv(R+Bt'*Pt*Bt)*Bt'*Pt*It;
Gx = inv(R + Bt'*Pt*Bt)*Bt'*Pt*Ft;
Gd(1) = Gi;
Gd(2:N) = inv(R + Bt'*Pt*Bt)*Bt'*Xtt(:,1:end-1);

% Kajita style g_j calculation
for i=1:N
    f(i) = inv(R+Bt'*Pt*Bt)*(Bt'*(((At_cl)'^(i-1))*Ct'))*Qe;
end

for i=1:N
    g(i) = sum(f(i:N));
end


% VERIFIED
% Xt = At'*(Pt*Vt+Xt*Ad) - At'*Pt*Bt*inv(R+Bt'*Pt*Bt)*Bt'*(Pt*Vt+Xt*Ad)

Gi;
Gx;
Gd;

figure
plot(g)

% %%
% Gp = [];
% for i=1:N
%     Gp(end+1) = (R + Bt'*P*Bt)\Bt'*(At-Bt*Pt)'^i*Ct'*Qe;
% end

%%

% simulating x0
x0 = [0 0 0]';
q = [zeros(1,p), x0']';

%%

%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Method 1 %
%%%%%%%%%%%%%%%%%%%%%%%

G = [Gi Gx]
u = []
y = Ct*q;
for i=1:length(pxref)-N-1
    u(end+1) = -G*q(:,i) +  g*pxref(i:i+N-1)';
    q(:,end+1) = At*q(:,i) + Bt*u(:,i) - It*pxref(i+1);
    y(:,end+1) = C*q(p+1:end,i);
end
    
figure
subplot(2,4,1:4)
hold all
plot(t(1:size(y,2)),pxref(1:size(y,2)))
plot(t(1:size(y,2)),y)
plot(t(1:size(y,2)),q(2,:),'r-')
legend('pxref','zmp','x CoM')
title('x xmp')
xlabel('time [s]')
ylabel('x [m]')
subplot(2,4,5:8)
plot(t(1:length(u)),u)
title('control input')

%%
% simulating y0

x0 = [0 0 0]';
q = [zeros(1,p),x0']';

G = [Gi Gx]'
u = []
y = Ct*q;

for i=1:length(pyref)-N-1
    u(end+1) = -Ktilde*q(:,i) +  g*pyref(i:i+N-1)';
    q(:,end+1) = At*q(:,i) + Bt*u(:,i) - It*pyref(i+1);
    y(:,end+1) = C*q(p+1:end,i);
end
    
figure
subplot(2,4,1:4)
hold all
plot(t(1:size(y,2)),pyref(1:size(y,2)))
plot(t(1:size(y,2)),y)
plot(t(1:size(y,2)),q(2,:),'r-')
legend('pyref','zmp','y CoM')
title('y xmp')
xlabel('time [s]')
ylabel('x [m]')
subplot(2,4,5:8)
plot(t(1:length(u)),u)
title('control input')


% %%
% 
% %%%%%%%%%%%%%%%%%%%%%%%
% % Simulation Method 2 %
% %%%%%%%%%%%%%%%%%%%%%%%
% 
% q_kaj = x0;
% y_kaj = C*q_kaj;
% for i=1:length(pxref)-N-1
%     u(i) = -G'*q(:,i) + g*pxref(i:i+N-1)';
%     q_kaj(:,i+1) = A*q_kaj(:,i) + B*u(:,i);
%     y_kaj(:,i+1) = C*q_kaj(:,i);
% end
% 
% 
% figure
% hold on
% plot(t(1:length(y_kaj)),pxref(1:length(y_kaj)))
% plot(t(1:length(y_kaj)),y_kaj)
% plot(t(1:length(y_kaj)),q_kaj(1,:))
% 
% 
% x = 0
% for i=1:size(q_kaj,2)
%     x(i+1) = x(i) + q_kaj(i);
% end
% % plot(x)
% 
% %%
% 
% %%%%%%%%%%%%%%%%%%%%%%%
% % Simulation Method 3 %
% %%%%%%%%%%%%%%%%%%%%%%%

