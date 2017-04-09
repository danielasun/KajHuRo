function [ q,y,u ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,yref,x0,err0,previewLength)
%SIMULATE_PREVIEW_DYNAMICS
%  [ q,y,u ] = SimulatePreviewDynamics(sys,sys_t,Gi,Gx,Gd,yref,x0,err0,previewLength)
%
%   INPUTS:
%   sys = original system
%   sys_t = sys tilde, the extended linear system. Ref Katayama preview control(1985)
%   Gi = Optimal gain on the error state
%   Gx = Optimal gain on the state error
%   Gd = Preview gain
%   x0 = initial condition. 
%   err0 = initial amount of error in y to start out with. If you're starting
%   out, a good guess for this is usually zero.
%   previewLength = number of time steps in preview
%   
%   OUTPUTS:
%   q = vector of states for each time step
%   y = vector of outputs for each time step for original system (not yt)
%   u = vector of control inputs for each time step

    At = sys_t.a;
    Bt = sys_t.b;
    Ct = sys_t.c;
    
    C = sys.c;

    p = size(yref,1); % dimension of output state y

    %% simulating x0
    q = [err0, x0']';
    G = [Gi Gx];
    u = [];
    y = C*x0; % changed to C instead of Ct
    It = [eye(p);zeros(size(Bt,1)-p,size(Bt,2))];

%     if nStepsToSimulate > length(yref)-previewLength-1
%         error 'ERROR: number of steps to simulate must be less than length(yref)-previewLength-1'
%     elseif nStepsToSimulate == -1
%         nStepsToSimulate = length(yref)-previewLength-1
%     end
%     
    for i=1:length(yref)-previewLength-1
        u(end+1) = -G*q(:,i) +  Gd*yref(i:i+previewLength-1)';
        q(:,end+1) = At*q(:,i) + Bt*u(:,i) - It*yref(i+1);
        y(:,end+1) = C*q(p+1:end,i);
    end
    
end

