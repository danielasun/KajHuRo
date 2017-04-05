function [sys_t,Gi,Gx,Gd] =  preview_control(sys,N,Qe,Qx,R)
% preview_ctrl
%   Preview Control based on the method in Kajita's Introduction to
%   Humanoid Robotics and Katayama(1985)
%   
%   Takes a discrete time system with a defined step size T and a number of
%   steps to look ahead N.
%
%   Positive definite matrices Qe and Qx describe weighting errors for the
%   cost function J = sigma( e(i)*Qe*e(i) + dx(i)*Qx*dx(i) + du(i)*R*du(i))

A = sys.a;
B = sys.b;
C = sys.c;
D = sys.d;
T = sys.ts;

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
[Kbar,Pbar,e] = dlqr(Abar,Bbar,Qbar,R,0);
Pt = Pbar(1:p+n,1:p+n);
Xt = Pbar(1:p+n,p+n+1:end);
Ktilde = inv(R+Bt'*Pt*Bt)*Bt'*Pt*At;
At_cl = At-Bt*inv(R+Bt'*Pt*Bt)*Bt'*Pt*At;

Zt = Pbar(p+n+1:end,p+n+1:end);

Gi = inv(R+Bt'*Pt*Bt)*Bt'*Pt*It;
Gx = inv(R + Bt'*Pt*Bt)*Bt'*Pt*Ft;
 
% TODO: these lines for the preview gain I'm not sure of; Katayama appears to say that they should be
% negative, but Kajita's gains are positive and they fully work.
Gd(1) = Gi;
Gd(2:N) = -inv(R + Bt'*Pt*Bt)*Bt'*Xt(:,1:end-1);

sys_t = ss(At,Bt,Ct,0,T);

end
